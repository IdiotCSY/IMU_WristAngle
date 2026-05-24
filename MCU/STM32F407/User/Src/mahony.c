#include "mahony.h"
#include <math.h>

/*
 * ===================== 内部辅助函数 =====================
 */

static float Mahony_Clamp(float x, float min_val, float max_val)
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
 * ===================== 参数与初始化 =====================
 */

MahonyParams_t Mahony_DefaultParams(void)
{
    MahonyParams_t p;

    /*
     * 这组参数对应你在 MATLAB 实物测试中后面调得效果较好的版本。
     */
    p.Kp = 1.5f;
    p.Ki = 0.0f;

    p.g = 9.81f;

    p.accTol = 1.0f;        /* m/s^2 */
    p.gyroTolDeg = 120.0f; /* deg/s */

    p.intLimit = 0.5f;

    return p;
}

void Mahony_Init(MahonyFilter_t *filter)
{
    if (filter == 0)
    {
        return;
    }

    filter->q = Quat_Identity();
    filter->intErr = Vec3f_Create(0.0f, 0.0f, 0.0f);

    filter->accWeight = 0.0f;
    filter->error = Vec3f_Create(0.0f, 0.0f, 0.0f);
    filter->accNorm = 0.0f;
    filter->gyroNormDeg = 0.0f;
}

void Mahony_Reset(MahonyFilter_t *filter)
{
    /*
     * 这个函数是“强制重置滤波器内部状态”。
     *
     * 注意：
     *   对 MH 来说，运行过程中不要频繁调用它做实验零位。
     *   因为重置 q 后，acc 会把 q 重新拉回重力方向，
     *   可能产生你之前见到的 0.5° 左右初始误差。
     *
     * 更合理的双 IMU 置零方式是：
     *
     *   qRel0 = qArm^{-1} ⊗ qHand
     *
     * 这个逻辑后面放在 dual_imu.c 里。
     */
    Mahony_Init(filter);
}

/*
 * ===================== 核心更新函数 =====================
 */

void Mahony_UpdateAdaptive(MahonyFilter_t *filter,
                           Vec3f_t gyro_rad_s,
                           Vec3f_t acc_mps2,
                           float dt,
                           const MahonyParams_t *params)
{
    if (filter == 0 || params == 0)
    {
        return;
    }

    /*
     * dt 异常时，不更新。
     */
    if (dt <= 0.0f)
    {
        return;
    }

    /*
     * 1. 当前状态读取与归一化
     */
    Quat_t q = Quat_Normalize(filter->q);

    /*
     * 2. 计算加速度模长和角速度模长
     */
    float accNorm = Vec3f_Norm(acc_mps2);
    float gyroNormRad = Vec3f_Norm(gyro_rad_s);
    float gyroNormDeg = gyroNormRad * QUAT_RAD_TO_DEG;

    filter->accNorm = accNorm;
    filter->gyroNormDeg = gyroNormDeg;

    /*
     * 3. 默认不使用加速度修正。
     */
    float accWeight = 0.0f;
    Vec3f_t e = Vec3f_Create(0.0f, 0.0f, 0.0f);

    /*
     * 4. 判断加速度是否可信。
     *
     * acc 只有在低动态时才近似等于重力。
     * 如果手动快速晃动，acc 中会包含线加速度、离心加速度等，
     * 这时不能强行用 acc 修正姿态。
     */
    if (accNorm > 1.0e-6f)
    {
        /*
         * accUnit 是加速度计测到的方向。
         * 静止时，它可以近似认为是重力方向在 body 坐标系中的表达。
         */
        Vec3f_t accUnit = Vec3f_Normalize(acc_mps2);

        /*
         * 当前 q 预测出来的重力方向。
         *
         * q 表示 body -> world，
         * 所以 R(q)^T * [0,0,1] 是世界 Z 轴在 body 中的表达。
         */
        Vec3f_t gHatBody = Quat_GetWorldZInBody(q);

        /*
         * 姿态误差。
         *
         * 这里和 MATLAB 版本保持一致：
         *
         *   e = accUnit × gHatBody
         *
         * 它表示“测得重力方向”和“预测重力方向”的偏差。
         */
        e = Vec3f_Cross(accUnit, gHatBody);

        /*
         * 4.1 加速度模长权重
         *
         * accNorm 越接近 g，说明 acc 越像纯重力，越可信。
         */
        float accErr = fabsf(accNorm - params->g);

        float wAccNorm = 1.0f - accErr / params->accTol;
        wAccNorm = Mahony_Clamp(wAccNorm, 0.0f, 1.0f);

        /*
         * 4.2 角速度权重
         *
         * 角速度越小，越说明当前接近静止或低动态，
         * 越适合用 acc 修正。
         */
        float wGyro = 1.0f - gyroNormDeg / params->gyroTolDeg;
        wGyro = Mahony_Clamp(wGyro, 0.0f, 1.0f);

        /*
         * 4.3 综合权重
         *
         * 这里采用平方，让权重更保守：
         *
         *   accWeight = (wAccNorm * wGyro)^2
         *
         * 当 acc 或 gyro 任一条件不可靠时，acc 修正都会明显减弱。
         */
        accWeight = wAccNorm * wGyro;
        accWeight = accWeight * accWeight;
    }

    filter->accWeight = accWeight;
    filter->error = e;

    /*
     * 5. 积分误差项
     *
     * 当前 Ki = 0，所以 intErr 对结果没有实际影响。
     * 但保留它，后续如果需要估计 gyro bias，可以打开 Ki。
     */
    Vec3f_t intErr = filter->intErr;

    intErr.x += accWeight * e.x * dt;
    intErr.y += accWeight * e.y * dt;
    intErr.z += accWeight * e.z * dt;

    intErr.x = Mahony_Clamp(intErr.x, -params->intLimit, params->intLimit);
    intErr.y = Mahony_Clamp(intErr.y, -params->intLimit, params->intLimit);
    intErr.z = Mahony_Clamp(intErr.z, -params->intLimit, params->intLimit);

    filter->intErr = intErr;

    /*
     * 6. 修正角速度
     *
     * 公式：
     *
     *   omega_corr = omega + Kp * accWeight * e + Ki * intErr
     *
     * 快速运动时 accWeight 接近 0，算法退化为纯 GI；
     * 低动态时 accWeight 增大，用 acc 修正 roll / pitch。
     */
    Vec3f_t omegaCorr;

    omegaCorr.x = gyro_rad_s.x
                  + params->Kp * accWeight * e.x
                  + params->Ki * intErr.x;

    omegaCorr.y = gyro_rad_s.y
                  + params->Kp * accWeight * e.y
                  + params->Ki * intErr.y;

    omegaCorr.z = gyro_rad_s.z
                  + params->Kp * accWeight * e.z
                  + params->Ki * intErr.z;

    /*
     * 7. 用修正后的角速度做四元数积分
     */
    q = Quat_IntegrateGyro(q, omegaCorr, dt);

    /*
     * 8. 保存新姿态
     */
    filter->q = Quat_Normalize(q);
}
