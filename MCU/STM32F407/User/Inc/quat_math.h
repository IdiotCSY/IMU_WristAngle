#ifndef __QUAT_MATH_H
#define __QUAT_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ===================== 模块说明 =====================
 *
 * 本模块负责姿态解算中最基础的数学运算：
 *
 *   1. 三维向量运算；
 *   2. 四元数运算；
 *   3. 陀螺仪积分；
 *   4. 四元数转欧拉角；
 *   5. 四元数转轴角；
 *   6. 根据姿态预测重力方向。
 *
 * 四元数约定：
 *
 *   q = [w, x, y, z]
 *
 * 其中：
 *   w 是标量部分；
 *   x, y, z 是向量部分。
 *
 * 姿态含义约定：
 *
 *   q 表示 body 坐标系到 world 坐标系的旋转。
 *
 * 也就是：
 *
 *   v_world = R(q) * v_body
 *
 * 后续 Mahony 算法中，需要把世界系重力方向转回 body 坐标系：
 *
 *   g_body = R(q)^T * g_world
 */

/* ===================== 基础常量 ===================== */

#ifndef QUAT_PI
#define QUAT_PI 3.14159265358979323846f
#endif

#define QUAT_RAD_TO_DEG  (180.0f / QUAT_PI)
#define QUAT_DEG_TO_RAD  (QUAT_PI / 180.0f)

/* ===================== 数据结构定义 ===================== */

/*
 * 三维向量结构体。
 *
 * 用结构体而不是 float[3] 的好处：
 *   1. 代码可读性更好；
 *   2. v.x / v.y / v.z 比 v[0] / v[1] / v[2] 更直观；
 *   3. 不容易把轴顺序写错。
 */
typedef struct
{
    float x;
    float y;
    float z;

} Vec3f_t;

/*
 * 四元数结构体。
 *
 * 约定：
 *   q.w = 标量部分；
 *   q.x, q.y, q.z = 向量部分。
 */
typedef struct
{
    float w;
    float x;
    float y;
    float z;

} Quat_t;

/*
 * ZYX 欧拉角结构体。
 *
 * 约定：
 *   roll  : 绕 X 轴旋转；
 *   pitch : 绕 Y 轴旋转；
 *   yaw   : 绕 Z 轴旋转。
 *
 * 单位：
 *   deg。
 */
typedef struct
{
    float roll;
    float pitch;
    float yaw;

} EulerDeg_t;

/*
 * 轴角结构体。
 *
 * axis:
 *   单位旋转轴。
 *
 * angle_deg:
 *   绕该轴旋转的总角度，单位 deg。
 */
typedef struct
{
    Vec3f_t axis;
    float angle_deg;

} AxisAngleDeg_t;

/* ===================== 向量函数 ===================== */

Vec3f_t Vec3f_Create(float x, float y, float z);

float Vec3f_Dot(Vec3f_t a, Vec3f_t b);
Vec3f_t Vec3f_Cross(Vec3f_t a, Vec3f_t b);

float Vec3f_Norm(Vec3f_t v);
Vec3f_t Vec3f_Normalize(Vec3f_t v);

/* ===================== 四元数函数 ===================== */

Quat_t Quat_Create(float w, float x, float y, float z);
Quat_t Quat_Identity(void);

float Quat_Norm(Quat_t q);
Quat_t Quat_Normalize(Quat_t q);
Quat_t Quat_Conj(Quat_t q);

/*
 * 四元数乘法：
 *
 *   q = q1 ⊗ q2
 *
 * 注意：
 *   四元数乘法不满足交换律。
 *   q1 ⊗ q2 和 q2 ⊗ q1 通常不一样。
 */
Quat_t Quat_Mul(Quat_t q1, Quat_t q2);

/*
 * 陀螺仪积分：
 *
 * 输入：
 *   q           : 当前姿态；
 *   gyro_rad_s  : 角速度，单位 rad/s，body 坐标系；
 *   dt          : 采样周期，单位 s。
 *
 * 输出：
 *   下一时刻姿态。
 */
Quat_t Quat_IntegrateGyro(Quat_t q, Vec3f_t gyro_rad_s, float dt);

/*
 * 四元数转 ZYX 欧拉角，单位 deg。
 */
EulerDeg_t Quat_ToEulerZYXDeg(Quat_t q);

/*
 * 四元数转轴角，单位 deg。
 */
AxisAngleDeg_t Quat_ToAxisAngleDeg(Quat_t q);

/*
 * 计算当前姿态下，世界系 Z 轴方向在 body 坐标系下的表达。
 *
 * 在 Mahony 中，如果世界系重力方向设为：
 *
 *   g_world = [0, 0, 1]
 *
 * 则该函数返回：
 *
 *   g_body = R(q)^T * g_world
 *
 * 它可以看作“当前姿态预测出来的加速度计重力方向”。
 */
Vec3f_t Quat_GetWorldZInBody(Quat_t q);

/*
 * 角度 wrap 到 [-180, 180]。
 */
float Quat_WrapTo180(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif
