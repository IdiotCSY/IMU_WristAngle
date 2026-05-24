#include "quat_math.h"
#include <math.h>

/*
 * ===================== 内部工具函数 =====================
 */

static float Quat_Clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }

    if (x > max_val)
    {
        return max_val;
    }

    return x;
}

/*
 * ===================== 向量函数 =====================
 */

Vec3f_t Vec3f_Create(float x, float y, float z)
{
    Vec3f_t v;

    v.x = x;
    v.y = y;
    v.z = z;

    return v;
}

float Vec3f_Dot(Vec3f_t a, Vec3f_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f_t Vec3f_Cross(Vec3f_t a, Vec3f_t b)
{
    Vec3f_t c;

    /*
     * 三维向量叉乘：
     *
     *   c = a × b
     *
     * 方向满足右手定则。
     */
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;

    return c;
}

float Vec3f_Norm(Vec3f_t v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3f_t Vec3f_Normalize(Vec3f_t v)
{
    float n = Vec3f_Norm(v);

    /*
     * 如果模长太小，说明这个向量接近 0。
     * 此时直接返回零向量，避免除以 0。
     */
    if (n < 1.0e-12f)
    {
        return Vec3f_Create(0.0f, 0.0f, 0.0f);
    }

    return Vec3f_Create(v.x / n, v.y / n, v.z / n);
}

/*
 * ===================== 四元数基础函数 =====================
 */

Quat_t Quat_Create(float w, float x, float y, float z)
{
    Quat_t q;

    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;

    return q;
}

Quat_t Quat_Identity(void)
{
    /*
     * 单位四元数表示“没有旋转”。
     */
    return Quat_Create(1.0f, 0.0f, 0.0f, 0.0f);
}

float Quat_Norm(Quat_t q)
{
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quat_t Quat_Normalize(Quat_t q)
{
    float n = Quat_Norm(q);

    /*
     * 四元数在积分过程中可能因为数值误差不再严格为单位长度。
     * 所以后续每次更新后都要归一化。
     *
     * 如果模长异常小，直接返回单位四元数，避免除以 0。
     */
    if (n < 1.0e-12f)
    {
        return Quat_Identity();
    }

    return Quat_Create(q.w / n, q.x / n, q.y / n, q.z / n);
}

Quat_t Quat_Conj(Quat_t q)
{
    /*
     * 单位四元数的共轭等于它的逆。
     *
     * 如果 q 表示 A -> B 的旋转，
     * 那么 conj(q) 表示 B -> A 的旋转。
     */
    return Quat_Create(q.w, -q.x, -q.y, -q.z);
}

Quat_t Quat_Mul(Quat_t q1, Quat_t q2)
{
    Quat_t q;

    /*
     * 四元数乘法：
     *
     *   q = q1 ⊗ q2
     *
     * 这个公式和 MATLAB 里之前写的 quat_mul 一致。
     */

    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;

    q.x = q1.w * q2.x + q2.w * q1.x + q1.y * q2.z - q1.z * q2.y;

    q.y = q1.w * q2.y + q2.w * q1.y + q1.z * q2.x - q1.x * q2.z;

    q.z = q1.w * q2.z + q2.w * q1.z + q1.x * q2.y - q1.y * q2.x;

    return q;
}

/*
 * ===================== 陀螺仪积分 =====================
 */

Quat_t Quat_IntegrateGyro(Quat_t q, Vec3f_t gyro_rad_s, float dt)
{
    /*
     * 纯陀螺积分逻辑：
     *
     *   当前角速度 gyro_rad_s 在 dt 时间内产生一个小旋转 dq。
     *   然后：
     *
     *      q_new = q ⊗ dq
     *
     * 这里 gyro_rad_s 是 body 坐标系下的角速度。
     */

    q = Quat_Normalize(q);

    float wx = gyro_rad_s.x;
    float wy = gyro_rad_s.y;
    float wz = gyro_rad_s.z;

    float omega_norm = sqrtf(wx * wx + wy * wy + wz * wz);
    float theta = omega_norm * dt;

    Quat_t dq;

    if (theta < 1.0e-12f)
    {
        /*
         * 小角度近似：
         *
         *   dq ≈ [1, 0.5 * omega * dt]
         *
         * 当角速度很小时，用这个形式可以避免除以很小的 omega_norm。
         */
        dq.w = 1.0f;
        dq.x = 0.5f * wx * dt;
        dq.y = 0.5f * wy * dt;
        dq.z = 0.5f * wz * dt;
    }
    else
    {
        /*
         * 一般情况：
         *
         *   旋转角 theta = |omega| * dt
         *   旋转轴 axis = omega / |omega|
         *
         *   dq = [cos(theta/2), axis * sin(theta/2)]
         */
        float half_theta = 0.5f * theta;
        float sin_half = sinf(half_theta);
        float cos_half = cosf(half_theta);

        float ax = wx / omega_norm;
        float ay = wy / omega_norm;
        float az = wz / omega_norm;

        dq.w = cos_half;
        dq.x = ax * sin_half;
        dq.y = ay * sin_half;
        dq.z = az * sin_half;
    }

    q = Quat_Mul(q, dq);
    q = Quat_Normalize(q);

    return q;
}

/*
 * ===================== 四元数转欧拉角 =====================
 */

EulerDeg_t Quat_ToEulerZYXDeg(Quat_t q)
{
    EulerDeg_t eul;

    q = Quat_Normalize(q);

    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    /*
     * 采用 ZYX 欧拉角顺序：
     *
     *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
     *
     * 输出：
     *   roll  : 绕 X 轴；
     *   pitch : 绕 Y 轴；
     *   yaw   : 绕 Z 轴。
     *
     * 公式与 MATLAB 版本 quat_to_eulZYX_deg 保持一致。
     */

    float R11 = 1.0f - 2.0f * (y * y + z * z);
    float R21 = 2.0f * (x * y + w * z);
    float R31 = 2.0f * (x * z - w * y);
    float R32 = 2.0f * (y * z + w * x);
    float R33 = 1.0f - 2.0f * (x * x + y * y);

    float pitch_arg = -R31;
    pitch_arg = Quat_Clamp(pitch_arg, -1.0f, 1.0f);

    float pitch = asinf(pitch_arg);
    float roll  = atan2f(R32, R33);
    float yaw   = atan2f(R21, R11);

    eul.roll  = Quat_WrapTo180(roll  * QUAT_RAD_TO_DEG);
    eul.pitch = Quat_WrapTo180(pitch * QUAT_RAD_TO_DEG);
    eul.yaw   = Quat_WrapTo180(yaw   * QUAT_RAD_TO_DEG);

    return eul;
}

/*
 * ===================== 四元数转轴角 =====================
 */

AxisAngleDeg_t Quat_ToAxisAngleDeg(Quat_t q)
{
    AxisAngleDeg_t aa;

    q = Quat_Normalize(q);

    /*
     * q 和 -q 表示同一个姿态。
     * 为了避免角度在 0~360 之间跳变，统一让 w >= 0。
     */
    if (q.w < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    float v_norm = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);

    float angle_rad = 2.0f * atan2f(v_norm, q.w);
    float angle_deg = angle_rad * QUAT_RAD_TO_DEG;

    if (v_norm < 1.0e-8f)
    {
        aa.axis = Vec3f_Create(0.0f, 0.0f, 0.0f);
        aa.angle_deg = 0.0f;
    }
    else
    {
        aa.axis = Vec3f_Create(q.x / v_norm, q.y / v_norm, q.z / v_norm);

        /*
         * 对于单位四元数，并且已经保证 w >= 0，
         * angle_deg 理论范围是 [0, 180]。
         */
        aa.angle_deg = Quat_Clamp(angle_deg, 0.0f, 180.0f);
    }

    return aa;
}

/*
 * ===================== 当前姿态下的重力方向预测 =====================
 */

Vec3f_t Quat_GetWorldZInBody(Quat_t q)
{
    /*
     * 功能：
     *   计算世界系 Z 轴 [0,0,1] 在 body 坐标系中的方向。
     *
     * 如果 q 表示 body -> world：
     *
     *   v_world = R * v_body
     *
     * 那么：
     *
     *   v_body = R^T * v_world
     *
     * 当 v_world = [0,0,1] 时，
     * R^T * [0,0,1] 等于 R 的第三行。
     */

    q = Quat_Normalize(q);

    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    Vec3f_t g_body;

    /*
     * 旋转矩阵 R 的第三行：
     *
     *   R31 = 2(xz - wy)
     *   R32 = 2(yz + wx)
     *   R33 = 1 - 2(x^2 + y^2)
     */
    g_body.x = 2.0f * (x * z - w * y);
    g_body.y = 2.0f * (y * z + w * x);
    g_body.z = 1.0f - 2.0f * (x * x + y * y);

    return Vec3f_Normalize(g_body);
}

/*
 * ===================== 角度 wrap =====================
 */

float Quat_WrapTo180(float angle_deg)
{
    /*
     * 把角度限制到 [-180, 180]。
     *
     * 例如：
     *   190  -> -170
     *   -190 -> 170
     */

    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}
