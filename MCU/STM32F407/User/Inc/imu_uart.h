#ifndef __IMU_UART_H
#define __IMU_UART_H

#include "main.h"
#include "wit_protocol.h"
#include "dual_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===================== 模块说明 =====================
 *
 * 串口分配：
 *
 *   USART1 -> PC / MATLAB
 *   USART2 -> hand IMU
 *   USART3 -> arm  IMU
 *
 * 当前功能：
 *
 *   1. USART2 接收 hand IMU 的 ACC + GYRO 数据；
 *   2. USART3 接收 arm  IMU 的 ACC + GYRO 数据；
 *   3. 调用 DualIMU 算法计算相对姿态；
 *   4. USART1 向 MATLAB 输出：
 *
 *        ATT,roll,pitch,yaw,theta,handCnt,armCnt,handBad,armBad
 *
 *   5. USART1 接收 MATLAB 发来的命令：
 *
 *        z : 将当前相对姿态定义为零位
 */

void IMU_UART_Init(void);
void IMU_UART_Task(void);

/* 手动置零接口，后续按键/串口命令都可以调用它 */
void IMU_UART_SetZero(void);

/* 获取底层解析器，主要用于调试 */
WIT_Parser_t* IMU_UART_GetHandParser(void);
WIT_Parser_t* IMU_UART_GetArmParser(void);

/* 获取双 IMU 姿态系统，主要用于调试 */
DualIMU_t* IMU_UART_GetDualIMU(void);

#ifdef __cplusplus
}
#endif

#endif
