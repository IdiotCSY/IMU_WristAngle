#ifndef __DUAL_IMU_H
#define __DUAL_IMU_H

#include "mahony.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===================== 模块说明 =====================
 *
 * 本模块负责“双 IMU 相对姿态计算”。
 *
 * 它不负责串口接收，也不负责 WIT 协议解析。
 *
 * 它只关心：
 *
 *   hand IMU 的 gyro + acc
 *   arm  IMU 的 gyro + acc
 *
 * 然后内部完成：
 *
 *   1. hand IMU 的 Mahony 姿态更新；
 *   2. arm  IMU 的 Mahony 姿态更新；
 *   3. 计算相对姿态：
 *
 *        qRel = qArm^{-1} ⊗ qHand
 *
 *   4. 计算零位补偿后的相对姿态：
 *
 *        qOut = qRel0^{-1} ⊗ qRel
 *
 *   5. 输出相对欧拉角和轴角。
 */

/*
 * 单个 IMU 的编号。
 */
typedef enum
{
    DUAL_IMU_HAND = 0,
    DUAL_IMU_ARM  = 1

} DualIMU_ID_t;

/*
 * 双 IMU 系统状态结构体。
 */
typedef struct
{
    /*
     * 两个 Mahony 滤波器。
     *
     * hand_filter:
     *   对应 hand IMU。
     *
     * arm_filter:
     *   对应 arm IMU。
     */
    MahonyFilter_t hand_filter;
    MahonyFilter_t arm_filter;

    /*
     * Mahony 参数。
     *
     * 两个 IMU 共用同一组参数。
     */
    MahonyParams_t params;

    /*
     * qRel:
     *   当前 hand 相对于 arm 的原始相对姿态。
     *
     * qOut:
     *   经过零位补偿后的相对姿态。
     *
     * qRel0:
     *   按 z 置零时记录下来的相对姿态。
     *   后续所有输出都相对于 qRel0。
     */
    Quat_t qRel;
    Quat_t qOut;
    Quat_t qRel0;

    /*
     * qHand0 / qArm0:
     *   按 z 时记录 hand / arm 当前姿态。
     *   主要用于后续如果要显示单个 IMU 相对变化。
     */
    Quat_t qHand0;
    Quat_t qArm0;

    /*
     * qHandShow / qArmShow:
     *   用于 3D 显示的姿态。
     *   它们表示相对于按 z 时刻的 hand / arm 姿态变化。
     */
    Quat_t qHandShow;
    Quat_t qArmShow;

    /*
     * 最终输出角度。
     *
     * relEuler:
     *   qOut 对应的 ZYX 欧拉角，单位 deg。
     *
     * relAxisAngle:
     *   qOut 对应的轴角，angle_deg 范围约为 0~180 deg。
     */
    EulerDeg_t relEuler;
    AxisAngleDeg_t relAxisAngle;

    /*
     * 是否已经执行过置零。
     *
     * 0:
     *   还没置零，qRel0 默认是单位四元数。
     *
     * 1:
     *   已经置零，qRel0 / qHand0 / qArm0 已经记录。
     */
    uint8_t zero_is_set;

} DualIMU_t;

/*
 * 初始化双 IMU 系统。
 */
void DualIMU_Init(DualIMU_t *sys);

/*
 * 设置 Mahony 参数。
 */
void DualIMU_SetParams(DualIMU_t *sys, MahonyParams_t params);

/*
 * 更新单个 IMU。
 *
 * 输入：
 *   id:
 *      DUAL_IMU_HAND 或 DUAL_IMU_ARM
 *
 *   gyro_deg_s:
 *      角速度，单位 deg/s
 *
 *   acc_mps2:
 *      加速度，单位 m/s^2
 *
 *   dt:
 *      采样周期，单位 s
 *
 * 注意：
 *   这里输入 gyro_deg_s 是为了和 WIT_Parser_t 的输出单位一致。
 *   函数内部会自动转换成 rad/s。
 */
void DualIMU_UpdateOne(DualIMU_t *sys,
                       DualIMU_ID_t id,
                       Vec3f_t gyro_deg_s,
                       Vec3f_t acc_mps2,
                       float dt);

/*
 * 更新相对姿态输出。
 *
 * 一般在 hand / arm 更新后调用。
 */
void DualIMU_UpdateRelative(DualIMU_t *sys);

/*
 * 将当前相对姿态定义为零位。
 *
 * 重要：
 *   这里不会把 hand_filter.q / arm_filter.q 重置为单位四元数。
 *   它只记录当前相对姿态 qRel0。
 *
 * 这和 MATLAB 版 MH_main 的新逻辑一致。
 */
void DualIMU_SetZero(DualIMU_t *sys);

/*
 * 获取 hand / arm 当前姿态。
 */
Quat_t DualIMU_GetHandQuat(const DualIMU_t *sys);
Quat_t DualIMU_GetArmQuat(const DualIMU_t *sys);

/*
 * 获取零位补偿后的相对姿态。
 */
Quat_t DualIMU_GetRelativeQuat(const DualIMU_t *sys);

/*
 * 获取相对欧拉角和轴角。
 */
EulerDeg_t DualIMU_GetRelativeEuler(const DualIMU_t *sys);
AxisAngleDeg_t DualIMU_GetRelativeAxisAngle(const DualIMU_t *sys);

#ifdef __cplusplus
}
#endif

#endif
