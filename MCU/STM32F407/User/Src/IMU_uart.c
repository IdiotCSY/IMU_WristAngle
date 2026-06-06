#include "imu_uart.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

/*
 * ===================== 文件逻辑说明 =====================
 *
 * 本文件负责把“串口通信”和“双 IMU 姿态算法”连接起来。
 *
 * 当前版本重点：
 *
 *   每解析到一个有效 GYRO 帧，就生成一个 IMU_Sample_t；
 *   主循环逐个取出 sample；
 *   每个 sample 都调用一次 DualIMU_UpdateOne()。
 *
 * 这样可以避免旧版中：
 *
 *   DMA 一次收到多帧，但主循环只用最新一帧更新一次
 *
 * 所导致的中间帧漏积分问题。
 */

/* ===================== 参数设置 ===================== */

/*
 * IMU DMA 接收缓冲区大小。
 *
 * WIT 一帧 11 字节。
 * ACC + GYRO 一组约 22 字节。
 */
#define IMU_RX_BUF_SIZE          256

/*
 * PC 命令接收缓冲区。
 * MATLAB 当前只发 z，64 字节足够。
 */
#define PC_RX_BUF_SIZE           64

/*
 * PC 输出字符串缓冲区。
 */
#define PC_TX_BUF_SIZE           256

/*
 * IMU 采样频率。
 *
 * 当前仍然使用固定 dt。
 * 如果维特上位机设置为 200 Hz，这里保持 200。
 * 如果设置为 100 Hz，这里改成 100。
 *
 * 下一步我们再用 TIM2 改成实测 dt。
 */
#define IMU_SAMPLE_FREQ_HZ       200.0f
#define IMU_SAMPLE_DT            (1.0f / IMU_SAMPLE_FREQ_HZ)

/*
 * STM32 向 MATLAB 输出频率。
 *
 * 20 ms 输出一次，大约 50 Hz。
 */
#define PC_OUTPUT_PERIOD_MS      20

/*
 * 每个 IMU 的 sample 队列长度。
 *
 * 200 Hz 下，64 个 sample 相当于 0.32 s 缓冲。
 * 正常主循环远快于这个速度，不应该溢出。
 */
#define IMU_SAMPLE_QUEUE_SIZE    64

#define DT_MONITOR_PERIOD_MS     1000U
#define TARGET_DT_US             (1000000.0f / IMU_SAMPLE_FREQ_HZ)

/* ===================== 数据结构定义 ===================== */

/*
 * 一个 IMU 姿态更新样本。
 *
 * 每解析到一个 GYRO 帧，就生成一个 sample。
 *
 * gyro_deg_s:
 *   当前 GYRO 帧对应的角速度，单位 deg/s。
 *
 * acc_mps2:
 *   最近一次解析到的加速度，单位 m/s^2。
 *
 * 说明：
 *   WIT 数据是分帧发的，ACC 和 GYRO 不在同一帧里。
 *   所以当 GYRO 帧到来时，使用“最近一次 ACC”配合它更新 Mahony。
 */
typedef struct
{
    Vec3f_t gyro_deg_s;
    Vec3f_t acc_mps2;

} IMU_Sample_t;

/*
 * 简单环形队列。
 *
 * push:
 *   在串口 DMA 回调中执行。
 *
 * pop:
 *   在主循环中执行。
 *
 * overflow_count:
 *   如果主循环处理不过来，队列满了，新样本会被丢弃，
 *   overflow_count 会增加。
 */
typedef struct
{
    IMU_Sample_t buf[IMU_SAMPLE_QUEUE_SIZE];

    uint16_t head;
    uint16_t tail;
    uint16_t count;

    uint32_t pushed_count;
    uint32_t popped_count;
    uint32_t overflow_count;

} IMU_SampleQueue_t;

/* ===================== 静态变量 ===================== */

/* USART2 / USART3 的 DMA 接收缓冲区 */
static uint8_t imu_hand_rx_buf[IMU_RX_BUF_SIZE];
static uint8_t imu_arm_rx_buf[IMU_RX_BUF_SIZE];

/* USART1 接收 MATLAB 命令的缓冲区 */
static uint8_t pc_rx_buf[PC_RX_BUF_SIZE];

/* WIT 协议解析器 */
static WIT_Parser_t wit_hand;
static WIT_Parser_t wit_arm;

/* 双 IMU 姿态系统 */
static DualIMU_t dual_imu;

/* hand / arm 各自的 sample 队列 */
static IMU_SampleQueue_t hand_queue;
static IMU_SampleQueue_t arm_queue;

/* PC 输出计时 */
static uint32_t last_output_tick = 0;

/* dt / 频率监测变量 */
static uint32_t last_dt_monitor_tick = 0;

static uint32_t last_hand_cnt_for_hz = 0;
static uint32_t last_arm_cnt_for_hz  = 0;

static float hand_hz = 0.0f;
static float arm_hz  = 0.0f;

static float hand_dt_avg_us = 0.0f;
static float arm_dt_avg_us  = 0.0f;

static float hand_dt_err_us = 0.0f;
static float arm_dt_err_us  = 0.0f;

/* ===================== DMA 单次接收监测变量 ===================== */

/*
 * 用来观察：
 *   1. 一次 DMA 回调收到了多少字节；
 *   2. 一次 DMA 回调中解析出了多少个 GYRO 帧；
 *   3. 平均每次 DMA 回调包含多少个 GYRO 帧；
 *   4. 最大一次 DMA 回调包含多少个 GYRO 帧。
 *
 * 目的：
 *   直观验证“逐 GYRO 帧更新 Mahony”相比旧逻辑的必要性。
 */

static uint32_t hand_dma_event_count = 0;
static uint32_t arm_dma_event_count  = 0;

static uint32_t hand_dma_total_gyro_frames = 0;
static uint32_t arm_dma_total_gyro_frames  = 0;

static uint16_t hand_dma_last_bytes = 0;
static uint16_t arm_dma_last_bytes  = 0;

static uint16_t hand_dma_last_gyro_frames = 0;
static uint16_t arm_dma_last_gyro_frames  = 0;

static uint16_t hand_dma_max_gyro_frames = 0;
static uint16_t arm_dma_max_gyro_frames  = 0;

static float hand_dma_avg_gyro_frames = 0.0f;
static float arm_dma_avg_gyro_frames  = 0.0f;

/* ===================== 内部函数声明 ===================== */

static void IMU_UART_StartReceive(void);
static void IMU_UART_RestartReceive(UART_HandleTypeDef *huart);

static void IMU_UART_UpdateAlgorithm(void);
static void IMU_UART_OutputToPC(void);

static void IMU_UART_ProcessPCCommand(const uint8_t *data, uint16_t len);
static void IMU_UART_Print(const char *str);

/* 队列相关函数 */
static void IMU_SampleQueue_Init(IMU_SampleQueue_t *q);
static void IMU_SampleQueue_Push(IMU_SampleQueue_t *q, IMU_Sample_t sample);
static uint8_t IMU_SampleQueue_Pop(IMU_SampleQueue_t *q, IMU_Sample_t *sample);

/* WIT 帧回调函数 */
static void IMU_UART_WitFrameCallbackHand(WIT_Parser_t *parser,
                                          uint8_t frame_type,
                                          void *user_data);

static void IMU_UART_WitFrameCallbackArm(WIT_Parser_t *parser,
                                         uint8_t frame_type,
                                         void *user_data);

static void IMU_UART_UpdateDtMonitor(void);

/* ===================== 对外函数实现 ===================== */

void IMU_UART_Init(void)
{
    //初始化计时变量
    last_dt_monitor_tick = HAL_GetTick();
    last_hand_cnt_for_hz = 0;
    last_arm_cnt_for_hz  = 0;

    hand_hz = 0.0f;
    arm_hz  = 0.0f;

    hand_dt_avg_us = 0.0f;
    arm_dt_avg_us  = 0.0f;

    hand_dt_err_us = 0.0f;
    arm_dt_err_us  = 0.0f;

    /*
     * 初始化两个 WIT 协议解析器。
     */
    WIT_Init(&wit_hand);
    WIT_Init(&wit_arm);

    /*
     * 初始化双 IMU 姿态系统。
     */
    DualIMU_Init(&dual_imu);

    /*
     * 初始化 hand / arm 两个 sample 队列。
     */
    IMU_SampleQueue_Init(&hand_queue);
    IMU_SampleQueue_Init(&arm_queue);

    last_output_tick = HAL_GetTick();

    /*
     * 启动 USART1 / USART2 / USART3 的 DMA + IDLE 接收。
     */
    IMU_UART_StartReceive();

    /*
     * 启动提示。
     * MATLAB 会忽略非 ATT 开头的行。
     */
    IMU_UART_Print("\r\n==============================\r\n");
    IMU_UART_Print("STM32 Dual IMU Attitude Start\r\n");
    IMU_UART_Print("Mode: per-gyro-frame Mahony update\r\n");
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
     * 主循环任务：
     *
     *   1. 从 hand / arm 队列逐个取 sample；
     *   2. 每个 sample 调用一次 Mahony；
     *   3. 更新相对姿态；
     *   4. 周期性输出到 MATLAB。
     */
    IMU_UART_UpdateAlgorithm();

    /* 更新实际采样频率 / 平均 dt 误差 */
    IMU_UART_UpdateDtMonitor();

    IMU_UART_OutputToPC();
}

void IMU_UART_SetZero(void)
{
    /*
     * 注意：
     *   这里不重置 hand_filter.q / arm_filter.q。
     *   只把当前 qRel 记录为零位 qRel0。
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        /*
         * USART1 收到 MATLAB 命令。
         */
        if (Size > 0)
        {
            IMU_UART_ProcessPCCommand(pc_rx_buf, Size);
        }

        IMU_UART_RestartReceive(&huart1);
    }
    else if (huart == &huart2)
    {
        /*
         * USART2 收到 hand IMU 数据。
         *
         * 这里使用新版 WIT_ParseBufferWithCallback。
         * 每解析到一个有效帧，都会进入 IMU_UART_WitFrameCallbackHand。
         */
        if (Size > 0)
        {
            /*
            * 记录解析前 hand 的 GYRO 帧计数。
            */
            uint32_t gyro_before = wit_hand.frame_count_gyro;

            /*
            * 解析本次 DMA 收到的一批字节。
            * 每解析到一个 GYRO 帧，会在回调中生成一个 sample 入队。
            */
            WIT_ParseBufferWithCallback(&wit_hand,
                                        imu_hand_rx_buf,
                                        Size,
                                        IMU_UART_WitFrameCallbackHand,
                                        0);

            /*
            * 解析后计算：本次 DMA 回调中实际解析出了多少个 GYRO 帧。
            */
            uint32_t gyro_after = wit_hand.frame_count_gyro;
            uint32_t gyro_delta = gyro_after - gyro_before;

            /*
            * 更新 DMA 监测信息。
            *
            * 如果 gyro_delta > 1，说明旧算法在这次 DMA 回调里会漏掉中间帧；
            * 当前新算法会通过 sample 队列逐帧处理。
            */
            hand_dma_event_count++;
            hand_dma_total_gyro_frames += gyro_delta;

            hand_dma_last_bytes = Size;
            hand_dma_last_gyro_frames = (uint16_t)gyro_delta;

            if (gyro_delta > hand_dma_max_gyro_frames)
            {
                hand_dma_max_gyro_frames = (uint16_t)gyro_delta;
            }

            if (hand_dma_event_count > 0)
            {
                hand_dma_avg_gyro_frames =
                    (float)hand_dma_total_gyro_frames / (float)hand_dma_event_count;
            }
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
            uint32_t gyro_before = wit_arm.frame_count_gyro;

            WIT_ParseBufferWithCallback(&wit_arm,
                                        imu_arm_rx_buf,
                                        Size,
                                        IMU_UART_WitFrameCallbackArm,
                                        0);

            uint32_t gyro_after = wit_arm.frame_count_gyro;
            uint32_t gyro_delta = gyro_after - gyro_before;

            arm_dma_event_count++;
            arm_dma_total_gyro_frames += gyro_delta;

            arm_dma_last_bytes = Size;
            arm_dma_last_gyro_frames = (uint16_t)gyro_delta;

            if (gyro_delta > arm_dma_max_gyro_frames)
            {
                arm_dma_max_gyro_frames = (uint16_t)gyro_delta;
            }

            if (arm_dma_event_count > 0)
            {
                arm_dma_avg_gyro_frames =
                    (float)arm_dma_total_gyro_frames / (float)arm_dma_event_count;
            }
        }

        IMU_UART_RestartReceive(&huart3);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /*
     * 如果出现串口错误，重新启动对应接收。
     */
    if (huart == &huart1 || huart == &huart2 || huart == &huart3)
    {
        IMU_UART_RestartReceive(huart);
    }
}

/* ===================== WIT 帧回调函数 ===================== */

static void IMU_UART_WitFrameCallbackHand(WIT_Parser_t *parser,
                                          uint8_t frame_type,
                                          void *user_data)
{
    (void)user_data;

    /*
     * 每解析到一个 hand GYRO 帧，就生成一个 sample。
     *
     * 注意：
     *   parser->gyro_deg_s 是刚刚更新的当前 GYRO；
     *   parser->acc_mps2 是最近一次 ACC。
     */
    if (frame_type == WIT_TYPE_GYRO)
    {
        IMU_Sample_t sample;

        sample.gyro_deg_s.x = parser->gyro_deg_s[0];
        sample.gyro_deg_s.y = parser->gyro_deg_s[1];
        sample.gyro_deg_s.z = parser->gyro_deg_s[2];

        sample.acc_mps2.x = parser->acc_mps2[0];
        sample.acc_mps2.y = parser->acc_mps2[1];
        sample.acc_mps2.z = parser->acc_mps2[2];

        IMU_SampleQueue_Push(&hand_queue, sample);
    }
}

static void IMU_UART_WitFrameCallbackArm(WIT_Parser_t *parser,
                                         uint8_t frame_type,
                                         void *user_data)
{
    (void)user_data;

    /*
     * 每解析到一个 arm GYRO 帧，就生成一个 sample。
     */
    if (frame_type == WIT_TYPE_GYRO)
    {
        IMU_Sample_t sample;

        sample.gyro_deg_s.x = parser->gyro_deg_s[0];
        sample.gyro_deg_s.y = parser->gyro_deg_s[1];
        sample.gyro_deg_s.z = parser->gyro_deg_s[2];

        sample.acc_mps2.x = parser->acc_mps2[0];
        sample.acc_mps2.y = parser->acc_mps2[1];
        sample.acc_mps2.z = parser->acc_mps2[2];

        IMU_SampleQueue_Push(&arm_queue, sample);
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
     * DMA Normal 模式下，每次回调后必须重新启动接收。
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
     * MATLAB 可能发送：
     *   z
     *   z\r\n
     *
     * 所以这里逐字节扫描，只要发现 z 或 Z 就置零。
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
     * 新版核心：
     *
     *   不再通过 gyro_count 判断“有没有新数据”；
     *   而是从 hand / arm 队列里逐个取 sample。
     *
     * 每取出一个 sample，就调用一次 DualIMU_UpdateOne()。
     */

    IMU_Sample_t sample;

    /*
     * 处理 hand 队列。
     *
     * 如果队列里有多个 sample，会在一次主循环里全部处理完。
     */
    while (IMU_SampleQueue_Pop(&hand_queue, &sample))
    {
        DualIMU_UpdateOne(&dual_imu,
                          DUAL_IMU_HAND,
                          sample.gyro_deg_s,
                          sample.acc_mps2,
                          IMU_SAMPLE_DT);
    }

    /*
     * 处理 arm 队列。
     */
    while (IMU_SampleQueue_Pop(&arm_queue, &sample))
    {
        DualIMU_UpdateOne(&dual_imu,
                          DUAL_IMU_ARM,
                          sample.gyro_deg_s,
                          sample.acc_mps2,
                          IMU_SAMPLE_DT);
    }

    /*
     * 根据最新 qHand / qArm 更新相对姿态。
     */
    DualIMU_UpdateRelative(&dual_imu);
}

static void IMU_UART_OutputToPC(void)
{
    uint32_t now = HAL_GetTick();

    if (now - last_output_tick < PC_OUTPUT_PERIOD_MS)
    {
        return;
    }

    last_output_tick = now;

    EulerDeg_t eul = DualIMU_GetRelativeEuler(&dual_imu);
    AxisAngleDeg_t aa = DualIMU_GetRelativeAxisAngle(&dual_imu);

    /*
     * 复制 WIT 计数器。
     * 这些计数器在中断回调里会更新，所以复制时短暂关中断。
     */
    WIT_Parser_t hand_copy;
    WIT_Parser_t arm_copy;

    uint32_t hand_overflow;
    uint32_t arm_overflow;

    __disable_irq();
    hand_copy = wit_hand;
    arm_copy  = wit_arm;
    hand_overflow = hand_queue.overflow_count;
    arm_overflow  = arm_queue.overflow_count;
    __enable_irq();

    char tx[PC_TX_BUF_SIZE];

    /*
     * 继续保持前 9 个字段不变，保证 MATLAB 旧脚本还能解析。
     *
     * 额外追加两个字段：
     *   handOverflow
     *   armOverflow
     *
     * MATLAB 当前只读取前 8 个数字，所以不会受影响。
     */
    snprintf(tx, sizeof(tx),
            "ATT,%.3f,%.3f,%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,%lu,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u,%.2f,%.2f,%u,%u\r\n",
            eul.roll,
            eul.pitch,
            eul.yaw,
            aa.angle_deg,
            (unsigned long)hand_copy.frame_count_gyro,
            (unsigned long)arm_copy.frame_count_gyro,
            (unsigned long)hand_copy.frame_count_bad,
            (unsigned long)arm_copy.frame_count_bad,
            (unsigned long)hand_overflow,
            (unsigned long)arm_overflow,
            hand_hz,
            arm_hz,
            hand_dt_err_us,
            arm_dt_err_us,

            /* 新增：DMA 单次接收监测 */
            hand_dma_last_bytes,
            arm_dma_last_bytes,
            hand_dma_last_gyro_frames,
            arm_dma_last_gyro_frames,
            hand_dma_avg_gyro_frames,
            arm_dma_avg_gyro_frames,
            hand_dma_max_gyro_frames,
            arm_dma_max_gyro_frames);

    IMU_UART_Print(tx);
}

static void IMU_UART_Print(const char *str)
{
    /*
     * 当前输出频率 50 Hz 左右，阻塞发送可以接受。
     * 以后如果输出更高频，再考虑 USART1_TX DMA。
     */
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

/* ===================== 队列函数实现 ===================== */

static void IMU_SampleQueue_Init(IMU_SampleQueue_t *q)
{
    if (q == 0)
    {
        return;
    }

    q->head = 0;
    q->tail = 0;
    q->count = 0;

    q->pushed_count = 0;
    q->popped_count = 0;
    q->overflow_count = 0;
}

static void IMU_SampleQueue_Push(IMU_SampleQueue_t *q, IMU_Sample_t sample)
{
    if (q == 0)
    {
        return;
    }

    /*
     * 如果队列满了，当前 sample 丢弃。
     * overflow_count 增加，后续可以通过 MATLAB 观察。
     */
    if (q->count >= IMU_SAMPLE_QUEUE_SIZE)
    {
        q->overflow_count++;
        return;
    }

    q->buf[q->head] = sample;

    q->head++;
    if (q->head >= IMU_SAMPLE_QUEUE_SIZE)
    {
        q->head = 0;
    }

    q->count++;
    q->pushed_count++;
}

static uint8_t IMU_SampleQueue_Pop(IMU_SampleQueue_t *q, IMU_Sample_t *sample)
{
    uint8_t ok = 0;

    if (q == 0 || sample == 0)
    {
        return 0;
    }

    /*
     * Pop 在主循环中执行，Push 可能在串口中断中执行。
     * 为了防止 pop 到一半被中断打断，这里短暂关中断。
     *
     * 关中断区域很短，只做几个变量读写。
     */
    __disable_irq();

    if (q->count > 0)
    {
        *sample = q->buf[q->tail];

        q->tail++;
        if (q->tail >= IMU_SAMPLE_QUEUE_SIZE)
        {
            q->tail = 0;
        }

        q->count--;
        q->popped_count++;

        ok = 1;
    }

    __enable_irq();

    return ok;
}

static void IMU_UART_UpdateDtMonitor(void)
{
    /*
     * 这个函数每 1 秒统计一次：
     *
     *   hand / arm 在这一秒内分别收到了多少个 GYRO 帧
     *
     * 然后换算：
     *
     *   实际频率 Hz
     *   平均 dt us
     *   平均 dt 误差 us
     *
     * 注意：
     *   这里测的是“窗口平均 dt”，不是单帧瞬时 dt。
     */

    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - last_dt_monitor_tick;

    if (elapsed_ms < DT_MONITOR_PERIOD_MS)
    {
        return;
    }

    uint32_t hand_cnt_now;
    uint32_t arm_cnt_now;

    /*
     * wit_hand / wit_arm 会在串口回调中更新。
     * 这里短暂关中断复制计数。
     */
    __disable_irq();
    hand_cnt_now = wit_hand.frame_count_gyro;
    arm_cnt_now  = wit_arm.frame_count_gyro;
    __enable_irq();

    uint32_t hand_delta = hand_cnt_now - last_hand_cnt_for_hz;
    uint32_t arm_delta  = arm_cnt_now  - last_arm_cnt_for_hz;

    float elapsed_s = (float)elapsed_ms / 1000.0f;

    if (elapsed_s > 0.0f)
    {
        hand_hz = (float)hand_delta / elapsed_s;
        arm_hz  = (float)arm_delta  / elapsed_s;

        if (hand_hz > 1.0f)
        {
            hand_dt_avg_us = 1000000.0f / hand_hz;
            hand_dt_err_us = hand_dt_avg_us - TARGET_DT_US;
        }

        if (arm_hz > 1.0f)
        {
            arm_dt_avg_us = 1000000.0f / arm_hz;
            arm_dt_err_us = arm_dt_avg_us - TARGET_DT_US;
        }
    }

    last_hand_cnt_for_hz = hand_cnt_now;
    last_arm_cnt_for_hz  = arm_cnt_now;
    last_dt_monitor_tick = now;
}
