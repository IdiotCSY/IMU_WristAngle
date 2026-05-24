#ifndef __MAHONY_H
#define __MAHONY_H

#include "quat_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===================== 模块说明 =====================
 *
 * Mahony / MH 算法可以理解为：
 *
 *   纯陀螺积分 GI + 加速度计重力方向修正
 *
 * 其中：
 *   1. gyro 负责快速动态响应；
 *   2. acc 在低动态时修正 roll / pitch 漂移；
 *   3. yaw 因为缺少磁力计或外部航向参考，不能从根本上被 acc 修正。
 *
 * 本模块对应 MATLAB 中的：
 *
 *   mahony_update_adaptive.m
 */

/*
 * Mahony 参数结构体。
 */
typedef struct
{
    /*
     * 比例修正系数。
     * Kp 越大，加速度计把姿态拉回重力方向越快。
     * 过大时，动态加速度会把姿态拉歪。
     */
    float Kp;

    /*
     * 积分修正系数。
     * 主要用于估计陀螺零偏。
     * 目前 WT-IMU63 静止 gyro 已经接近 0，所以第一版 Ki = 0。
     */
    float Ki;

    /*
     * 标准重力加速度，单位 m/s^2。
     */
    float g;

    /*
     * 加速度模长容差。
     *
     * 如果 |norm(acc) - g| 很大，说明 acc 中含有较强动态加速度，
     * 此时不适合用 acc 修正姿态。
     */
    float accTol;

    /*
     * 角速度阈值，单位 deg/s。
     *
     * 角速度越大，越说明当前处于快速动态过程，
     * 此时降低 acc 修正权重。
     */
    float gyroTolDeg;

    /*
     * 积分项限幅。
     * 防止 intErr 长时间累积过大。
     */
    float intLimit;

} MahonyParams_t;

/*
 * Mahony 滤波器状态结构体。
 */
typedef struct
{
    /*
     * 当前姿态四元数，约定为 body -> world。
     */
    Quat_t q;

    /*
     * 积分误差项。
     * 当前 Ki = 0 时基本不起作用，但保留接口。
     */
    Vec3f_t intErr;

    /*
     * 调试信息。
     */
    float accWeight;
    Vec3f_t error;
    float accNorm;
    float gyroNormDeg;

} MahonyFilter_t;

/*
 * 设置推荐初始参数。
 *
 * 当前使用你实测效果较好的参数：
 *   Kp = 1.5
 *   Ki = 0
 *   accTol = 1.0
 *   gyroTolDeg = 120
 *   intLimit = 0.5
 */
MahonyParams_t Mahony_DefaultParams(void);

/*
 * 初始化滤波器状态。
 */
void Mahony_Init(MahonyFilter_t *filter);

/*
 * 重置滤波器状态为单位四元数。
 *
 * 注意：
 *   对双 IMU 相对测角来说，后续按 z 置零时通常不要重置 filter->q。
 *   更推荐记录当前相对姿态为 qRel0。
 */
void Mahony_Reset(MahonyFilter_t *filter);

/*
 * 更新一次 Mahony 滤波器。
 *
 * 输入：
 *   filter      : 滤波器状态；
 *   gyro_rad_s  : 角速度，单位 rad/s，body 坐标系；
 *   acc_mps2    : 加速度，单位 m/s^2，body 坐标系；
 *   dt          : 采样周期，单位 s；
 *   params      : 参数结构体。
 */
void Mahony_UpdateAdaptive(MahonyFilter_t *filter,
                           Vec3f_t gyro_rad_s,
                           Vec3f_t acc_mps2,
                           float dt,
                           const MahonyParams_t *params);

#ifdef __cplusplus
}
#endif

#endif
