#include "imu_uart.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

/*
 * ===================== 文件逻辑说明 =====================
 *
 * 这个文件负责把“串口通信”和“双 IMU 姿态算法”连接起来。
 *
 * 数据主链路：
 *
 *   hand IMU
 *      ↓ USART2 + DMA
 *   WIT_ParseBuffer()
 *      ↓ 得到 hand 的 acc / gyro
 *   DualIMU_UpdateOne(DUAL_IMU_HAND)
 *
 *   arm IMU
 *      ↓ USART3 + DMA
 *   WIT_ParseBuffer()
 *      ↓ 得到 arm 的 acc / gyro
 *   DualIMU_UpdateOne(DUAL_IMU_ARM)
 *
 *   DualIMU_UpdateRelative()
 *      ↓ 得到相对姿态 roll / pitch / yaw / theta
 *
 *   USART1
 *      ↓ 输出给 MATLAB
 *   ATT,roll,pitch,yaw,theta,handCnt,armCnt,handBad,armBad
 *
 *
 * 命令链路：
 *
 *   MATLAB 发送字符 'z'
 *      ↓ USART1 + DMA
 *   HAL_UARTEx_RxEventCallback()
 *      ↓
 *   IMU_UART_ProcessPCCommand()
 *      ↓
 *   IMU_UART_SetZero()
 *      ↓
 *   DualIMU_SetZero()
 *
 *
 * 注意：
 *
 *   1. USART2/3 是 IMU 输入；
 *   2. USART1 是 PC/MATLAB 通信；
 *   3. 当前使用 DMA Normal + ReceiveToIdle；
 *   4. Normal DMA 每次回调后必须重新启动接收；
 *   5. 不要在中断回调里大量打印。
 */

/* ===================== 参数设置 ===================== */

/*
 * IMU DMA 接收缓冲区大小。
 *
 * 维特 IMU 一帧 11 字节。
 * 如果输出 ACC + GYRO，则每个采样点约 22 字节。
 * 256 字节足够第一版使用。
 */
#define IMU_RX_BUF_SIZE          256

/*
 * PC 命令接收缓冲区。
 *
 * MATLAB 现在只发 'z'，64 字节完全够。
 */
#define PC_RX_BUF_SIZE           64

/*
 * PC 输出字符串缓冲区。
 */
#define PC_TX_BUF_SIZE           256

/*
 * IMU 采样频率。
 *
 * 这里必须和维特上位机设置一致。
 *
 * 如果 IMU 设置 200 Hz：
 *   保持 200.0f
 *
 * 如果 IMU 设置 100 Hz：
 *   改成 100.0f
 */
#define IMU_SAMPLE_FREQ_HZ       200.0f
#define IMU_SAMPLE_DT            (1.0f / IMU_SAMPLE_FREQ_HZ)

/*
 * STM32 向 MATLAB 输出频率。
 *
 * 20 ms 输出一次，约 50 Hz。
 * 不要输出太快，否则 USART1 和 MATLAB 都会比较忙。
 */
#define PC_OUTPUT_PERIOD_MS      20

/* ===================== 静态变量 ===================== */

/*
 * USART2 / USART3 的 DMA 接收缓冲区。
 *
 * 它们只是临时接收一批串口字节。
 * 真正的解析状态保存在 wit_hand / wit_arm 中。
 */
static uint8_t imu_hand_rx_buf[IMU_RX_BUF_SIZE];
static uint8_t imu_arm_rx_buf[IMU_RX_BUF_SIZE];

/*
 * USART1 接收 MATLAB 命令的缓冲区。
 */
static uint8_t pc_rx_buf[PC_RX_BUF_SIZE];

/*
 * 维特协议解析器。
 *
 * wit_hand:
 *   对应 USART2，也就是 hand IMU。
 *
 * wit_arm:
 *   对应 USART3，也就是 arm IMU。
 */
static WIT_Parser_t wit_hand;
static WIT_Parser_t wit_arm;

/*
 * 双 IMU 姿态系统。
 *
 * 内部包含：
 *   hand Mahony 滤波器；
 *   arm  Mahony 滤波器；
 *   相对姿态 qRel / qOut；
 *   相对欧拉角 / 轴角。
 */
static DualIMU_t dual_imu;

/*
 * 上一次已经处理过的 gyro 帧计数。
 *
 * 用于判断是否有新的 gyro 数据需要送入算法。
 */
static uint32_t last_hand_gyro_count = 0;
static uint32_t last_arm_gyro_count  = 0;

/*
 * 上一次向 PC 输出 ATT 数据的时刻。
 */
static uint32_t last_output_tick = 0;

/* ===================== 内部函数声明 ===================== */

static void IMU_UART_StartReceive(void);
static void IMU_UART_RestartReceive(UART_HandleTypeDef *huart);

static void IMU_UART_UpdateAlgorithm(void);
static void IMU_UART_OutputToPC(void);

static void IMU_UART_ProcessPCCommand(const uint8_t *data, uint16_t len);
static void IMU_UART_Print(const char *str);

/* ===================== 对外函数实现 ===================== */

void IMU_UART_Init(void)
{
    /*
     * 初始化两个维特协议解析器。
     */
    WIT_Init(&wit_hand);
    WIT_Init(&wit_arm);

    /*
     * 初始化双 IMU 姿态系统。
     */
    DualIMU_Init(&dual_imu);

    /*
     * 初始化计数器。
     */
    last_hand_gyro_count = 0;
    last_arm_gyro_count  = 0;
    last_output_tick = HAL_GetTick();

    /*
     * 启动 USART1 / USART2 / USART3 的 DMA + IDLE 接收。
     */
    IMU_UART_StartReceive();

    /*
     * 启动提示信息。
     * MATLAB 会跳过非 ATT 开头的行，所以这些说明不会影响绘图。
     */
    IMU_UART_Print("\r\n==============================\r\n");
    IMU_UART_Print("STM32 Dual IMU Attitude Start\r\n");
    IMU_UART_Print("USART1 -> PC / MATLAB\r\n");
    IMU_UART_Print("USART2 -> hand IMU\r\n");
    IMU_UART_Print("USART3 -> arm  IMU\r\n");
    IMU_UART_Print("Output: ATT,roll,pitch,yaw,theta,handCnt,armCnt,handBad,armBad\r\n");
    IMU_UART_Print("Command: send 'z' to set zero\r\n");
    IMU_UART_Print("==============================\r\n\r\n");
}

void IMU_UART_Task(void)
{
    /*
     * 主循环里反复调用这个函数。
     *
     * 它做两件事：
     *   1. 如果收到新的 IMU gyro 数据，就更新姿态算法；
     *   2. 按固定周期向 MATLAB 输出姿态结果。
     */
    IMU_UART_UpdateAlgorithm();
    IMU_UART_OutputToPC();
}

void IMU_UART_SetZero(void)
{
    /*
     * 置零逻辑：
     *
     *   不重置 hand_filter.q / arm_filter.q；
     *   只把当前相对姿态记录为 qRel0。
     *
     * 这和 MATLAB 版 MH_main 后期修改后的逻辑一致。
     */
    DualIMU_SetZero(&dual_imu);

    IMU_UART_Print("ZERO SET\r\n");
}

WIT_Parser_t* IMU_UART_GetHandParser(void)
{
    return &wit_hand;
}

WIT_Parser_t* IMU_UART_GetArmParser(void)
{
    return &wit_arm;
}

DualIMU_t* IMU_UART_GetDualIMU(void)
{
    return &dual_imu;
}

/* ===================== HAL 回调函数 ===================== */

/*
 * HAL_UARTEx_RxEventCallback
 *
 * 使用 HAL_UARTEx_ReceiveToIdle_DMA() 后：
 *
 *   1. 串口出现 IDLE；
 *   2. 或 DMA buffer 收满；
 *
 * 就会进入这个回调。
 *
 * 参数 Size 表示本次实际收到的字节数。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        /*
         * USART1 收到 MATLAB 发来的命令。
         */
        if (Size > 0)
        {
            IMU_UART_ProcessPCCommand(pc_rx_buf, Size);
        }

        /*
         * DMA Normal 模式下，处理完必须重新启动接收。
         */
        IMU_UART_RestartReceive(&huart1);
    }
    else if (huart == &huart2)
    {
        /*
         * USART2 收到 hand IMU 数据。
         */
        if (Size > 0)
        {
            WIT_ParseBuffer(&wit_hand, imu_hand_rx_buf, Size);
        }

        IMU_UART_RestartReceive(&huart2);
    }
    else if (huart == &huart3)
    {
        /*
         * USART3 收到 arm IMU 数据。
         */
        if (Size > 0)
        {
            WIT_ParseBuffer(&wit_arm, imu_arm_rx_buf, Size);
        }

        IMU_UART_RestartReceive(&huart3);
    }
}

/*
 * 串口错误回调。
 *
 * 如果发生 ORE / FE / NE 等错误，
 * 重新启动对应串口接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 || huart == &huart2 || huart == &huart3)
    {
        IMU_UART_RestartReceive(huart);
    }
}

/* ===================== 内部函数实现 ===================== */

static void IMU_UART_StartReceive(void)
{
    /*
     * USART1 接收 MATLAB 命令。
     */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pc_rx_buf, PC_RX_BUF_SIZE);

    if (huart1.hdmarx != NULL)
    {
        /*
         * 关闭半传输中断。
         *
         * 我们只关心：
         *   1. IDLE；
         *   2. buffer 接收完成。
         */
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

    /*
     * USART2 接收 hand IMU。
     */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, imu_hand_rx_buf, IMU_RX_BUF_SIZE);

    if (huart2.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    /*
     * USART3 接收 arm IMU。
     */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, imu_arm_rx_buf, IMU_RX_BUF_SIZE);

    if (huart3.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

static void IMU_UART_RestartReceive(UART_HandleTypeDef *huart)
{
    /*
     * DMA Normal 模式下，每次回调后 DMA 会停止。
     * 因此要重新调用 HAL_UARTEx_ReceiveToIdle_DMA()。
     */

    if (huart == &huart1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pc_rx_buf, PC_RX_BUF_SIZE);

        if (huart1.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
        }
    }
    else if (huart == &huart2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, imu_hand_rx_buf, IMU_RX_BUF_SIZE);

        if (huart2.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }
    else if (huart == &huart3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, imu_arm_rx_buf, IMU_RX_BUF_SIZE);

        if (huart3.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }
}

static void IMU_UART_ProcessPCCommand(const uint8_t *data, uint16_t len)
{
    /*
     * MATLAB 发送命令可能不止一个字节。
     * 所以这里逐字节扫描。
     *
     * 当前支持：
     *   z / Z : 当前相对姿态置零
     */
    for (uint16_t i = 0; i < len; i++)
    {
        if (data[i] == 'z' || data[i] == 'Z')
        {
            IMU_UART_SetZero();
        }
    }
}

static void IMU_UART_UpdateAlgorithm(void)
{
    /*
     * 当前第一版策略：
     *
     *   如果 gyro 计数增加，就用最新 gyro + 最新 acc 更新一次。
     *
     * 这能先跑通 MCU 端完整链路。
     *
     * 注意：
     *   如果 DMA 一次收到多帧，而主循环只更新一次，
     *   理论上会漏掉中间部分积分。
     *
     * 后续更严谨版本会改成：
     *   每解析到一个 GYRO 帧，就立即更新一次算法。
     */

    WIT_Parser_t hand_copy;
    WIT_Parser_t arm_copy;

    /*
     * wit_hand / wit_arm 在串口中断回调里会被更新。
     * 这里短暂关中断复制一份，防止读到一半被修改。
     */
    __disable_irq();
    hand_copy = wit_hand;
    arm_copy  = wit_arm;
    __enable_irq();

    /*
     * hand IMU 有新的 gyro 数据。
     */
    if (hand_copy.frame_count_gyro != last_hand_gyro_count)
    {
        Vec3f_t gyro_deg_s;
        Vec3f_t acc_mps2;

        gyro_deg_s.x = hand_copy.gyro_deg_s[0];
        gyro_deg_s.y = hand_copy.gyro_deg_s[1];
        gyro_deg_s.z = hand_copy.gyro_deg_s[2];

        acc_mps2.x = hand_copy.acc_mps2[0];
        acc_mps2.y = hand_copy.acc_mps2[1];
        acc_mps2.z = hand_copy.acc_mps2[2];

        DualIMU_UpdateOne(&dual_imu,
                          DUAL_IMU_HAND,
                          gyro_deg_s,
                          acc_mps2,
                          IMU_SAMPLE_DT);

        last_hand_gyro_count = hand_copy.frame_count_gyro;
    }

    /*
     * arm IMU 有新的 gyro 数据。
     */
    if (arm_copy.frame_count_gyro != last_arm_gyro_count)
    {
        Vec3f_t gyro_deg_s;
        Vec3f_t acc_mps2;

        gyro_deg_s.x = arm_copy.gyro_deg_s[0];
        gyro_deg_s.y = arm_copy.gyro_deg_s[1];
        gyro_deg_s.z = arm_copy.gyro_deg_s[2];

        acc_mps2.x = arm_copy.acc_mps2[0];
        acc_mps2.y = arm_copy.acc_mps2[1];
        acc_mps2.z = arm_copy.acc_mps2[2];

        DualIMU_UpdateOne(&dual_imu,
                          DUAL_IMU_ARM,
                          gyro_deg_s,
                          acc_mps2,
                          IMU_SAMPLE_DT);

        last_arm_gyro_count = arm_copy.frame_count_gyro;
    }

    /*
     * 每次都更新相对姿态输出。
     */
    DualIMU_UpdateRelative(&dual_imu);
}

static void IMU_UART_OutputToPC(void)
{
    uint32_t now = HAL_GetTick();

    /*
     * 控制输出频率。
     */
    if (now - last_output_tick < PC_OUTPUT_PERIOD_MS)
    {
        return;
    }

    last_output_tick = now;

    EulerDeg_t eul = DualIMU_GetRelativeEuler(&dual_imu);
    AxisAngleDeg_t aa = DualIMU_GetRelativeAxisAngle(&dual_imu);

    WIT_Parser_t hand_copy;
    WIT_Parser_t arm_copy;

    /*
     * 复制计数器，用于输出时判断是否收包正常。
     */
    __disable_irq();
    hand_copy = wit_hand;
    arm_copy  = wit_arm;
    __enable_irq();

    char tx[PC_TX_BUF_SIZE];

    /*
     * CSV 格式输出。
     *
     * 字段含义：
     *
     *   ATT,
     *   roll,
     *   pitch,
     *   yaw,
     *   theta,
     *   handGyroCnt,
     *   armGyroCnt,
     *   handBad,
     *   armBad
     */
    snprintf(tx, sizeof(tx),
             "ATT,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%lu,%lu\r\n",
             eul.roll,
             eul.pitch,
             eul.yaw,
             aa.angle_deg,
             (unsigned long)hand_copy.frame_count_gyro,
             (unsigned long)arm_copy.frame_count_gyro,
             (unsigned long)hand_copy.frame_count_bad,
             (unsigned long)arm_copy.frame_count_bad);

    IMU_UART_Print(tx);
}

static void IMU_UART_Print(const char *str)
{
    /*
     * 当前输出频率约 50 Hz，阻塞发送可以接受。
     *
     * 后续如果要更高频输出，可以改成 USART1_TX DMA。
     */
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}
