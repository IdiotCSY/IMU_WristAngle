#include "dual_imu.h"

/*
 * ===================== 内部辅助函数 =====================
 */

/*
 * 将 Vec3f_t 从 deg/s 转换为 rad/s。
 *
 * WIT 模块解析出来的角速度单位是 deg/s；
 * Mahony / 四元数积分使用 rad/s。
 */
static Vec3f_t DualIMU_GyroDegToRad(Vec3f_t gyro_deg_s)
{
    Vec3f_t gyro_rad_s;

    gyro_rad_s.x = gyro_deg_s.x * QUAT_DEG_TO_RAD;
    gyro_rad_s.y = gyro_deg_s.y * QUAT_DEG_TO_RAD;
    gyro_rad_s.z = gyro_deg_s.z * QUAT_DEG_TO_RAD;

    return gyro_rad_s;
}

/*
 * 根据当前 hand / arm 滤波器状态，计算 qRel。
 *
 * 定义：
 *
 *   qRel = qArm^{-1} ⊗ qHand
 *
 * 其中：
 *   qHand 表示 hand body -> world；
 *   qArm  表示 arm  body -> world。
 *
 * qRel 表示 hand 相对于 arm 的姿态。
 */
static Quat_t DualIMU_ComputeRelativeQuat(Quat_t qHand, Quat_t qArm)
{
    Quat_t qRel;

    qRel = Quat_Mul(Quat_Conj(qArm), qHand);
    qRel = Quat_Normalize(qRel);

    return qRel;
}

/*
 * ===================== 对外函数实现 =====================
 */

void DualIMU_Init(DualIMU_t *sys)
{
    if (sys == 0)
    {
        return;
    }

    /*
     * 初始化两个 Mahony 滤波器。
     */
    Mahony_Init(&sys->hand_filter);
    Mahony_Init(&sys->arm_filter);

    /*
     * 使用默认 Mahony 参数。
     * 当前默认参数来自你 MATLAB 实测效果较好的那组。
     */
    sys->params = Mahony_DefaultParams();

    /*
     * 相对姿态相关变量初始化。
     */
    sys->qRel  = Quat_Identity();
    sys->qOut  = Quat_Identity();
    sys->qRel0 = Quat_Identity();

    sys->qHand0 = Quat_Identity();
    sys->qArm0  = Quat_Identity();

    sys->qHandShow = Quat_Identity();
    sys->qArmShow  = Quat_Identity();

    sys->relEuler.roll  = 0.0f;
    sys->relEuler.pitch = 0.0f;
    sys->relEuler.yaw   = 0.0f;

    sys->relAxisAngle.axis = Vec3f_Create(0.0f, 0.0f, 0.0f);
    sys->relAxisAngle.angle_deg = 0.0f;

    sys->zero_is_set = 0;
}

void DualIMU_SetParams(DualIMU_t *sys, MahonyParams_t params)
{
    if (sys == 0)
    {
        return;
    }

    sys->params = params;
}

void DualIMU_UpdateOne(DualIMU_t *sys,
                       DualIMU_ID_t id,
                       Vec3f_t gyro_deg_s,
                       Vec3f_t acc_mps2,
                       float dt)
{
    if (sys == 0)
    {
        return;
    }

    /*
     * dt 异常时直接返回。
     */
    if (dt <= 0.0f)
    {
        return;
    }

    /*
     * WIT 输出的 gyro 是 deg/s；
     * Mahony 使用 rad/s，所以这里做单位转换。
     */
    Vec3f_t gyro_rad_s = DualIMU_GyroDegToRad(gyro_deg_s);

    if (id == DUAL_IMU_HAND)
    {
        Mahony_UpdateAdaptive(&sys->hand_filter,
                              gyro_rad_s,
                              acc_mps2,
                              dt,
                              &sys->params);
    }
    else if (id == DUAL_IMU_ARM)
    {
        Mahony_UpdateAdaptive(&sys->arm_filter,
                              gyro_rad_s,
                              acc_mps2,
                              dt,
                              &sys->params);
    }
}

void DualIMU_UpdateRelative(DualIMU_t *sys)
{
    if (sys == 0)
    {
        return;
    }

    /*
     * 1. 取出当前 hand / arm 姿态。
     */
    Quat_t qHand = sys->hand_filter.q;
    Quat_t qArm  = sys->arm_filter.q;

    qHand = Quat_Normalize(qHand);
    qArm  = Quat_Normalize(qArm);

    /*
     * 2. 计算当前原始相对姿态：
     *
     *      qRel = qArm^{-1} ⊗ qHand
     */
    sys->qRel = DualIMU_ComputeRelativeQuat(qHand, qArm);

    /*
     * 3. 零位补偿：
     *
     *      qOut = qRel0^{-1} ⊗ qRel
     *
     * 如果还没置零，qRel0 是单位四元数，
     * 那么 qOut 就等于 qRel。
     */
    sys->qOut = Quat_Mul(Quat_Conj(sys->qRel0), sys->qRel);
    sys->qOut = Quat_Normalize(sys->qOut);

    /*
     * 4. 计算用于显示的 hand / arm 姿态。
     *
     * 如果已经按 z 置零，则显示相对于置零时刻的姿态变化；
     * 如果还没有置零，则直接显示当前姿态。
     */
    if (sys->zero_is_set)
    {
        sys->qHandShow = Quat_Mul(Quat_Conj(sys->qHand0), qHand);
        sys->qHandShow = Quat_Normalize(sys->qHandShow);

        sys->qArmShow = Quat_Mul(Quat_Conj(sys->qArm0), qArm);
        sys->qArmShow = Quat_Normalize(sys->qArmShow);
    }
    else
    {
        sys->qHandShow = qHand;
        sys->qArmShow  = qArm;
    }

    /*
     * 5. qOut 转换成欧拉角和轴角。
     */
    sys->relEuler = Quat_ToEulerZYXDeg(sys->qOut);
    sys->relAxisAngle = Quat_ToAxisAngleDeg(sys->qOut);
}

void DualIMU_SetZero(DualIMU_t *sys)
{
    if (sys == 0)
    {
        return;
    }

    /*
     * 先根据当前滤波器状态更新相对姿态。
     */
    DualIMU_UpdateRelative(sys);

    /*
     * 注意：
     *   这里不重置 hand_filter.q 和 arm_filter.q。
     *
     * 原因：
     *   MH 内部有加速度计修正。
     *   如果强制把 qHand / qArm 重置为单位四元数，
     *   它们会重新被 acc 拉向各自的重力方向，
     *   反而可能产生一个固定小误差。
     *
     * 正确做法：
     *   把当前相对姿态记录为零位。
     */
    sys->qRel0 = sys->qRel;

    /*
     * 记录单 IMU 显示零位。
     */
    sys->qHand0 = sys->hand_filter.q;
    sys->qArm0  = sys->arm_filter.q;

    sys->zero_is_set = 1;

    /*
     * 更新一次输出，让当前 qOut 立即变成接近单位四元数。
     */
    DualIMU_UpdateRelative(sys);
}

Quat_t DualIMU_GetHandQuat(const DualIMU_t *sys)
{
    if (sys == 0)
    {
        return Quat_Identity();
    }

    return sys->hand_filter.q;
}

Quat_t DualIMU_GetArmQuat(const DualIMU_t *sys)
{
    if (sys == 0)
    {
        return Quat_Identity();
    }

    return sys->arm_filter.q;
}

Quat_t DualIMU_GetRelativeQuat(const DualIMU_t *sys)
{
    if (sys == 0)
    {
        return Quat_Identity();
    }

    return sys->qOut;
}

EulerDeg_t DualIMU_GetRelativeEuler(const DualIMU_t *sys)
{
    EulerDeg_t e;

    if (sys == 0)
    {
        e.roll  = 0.0f;
        e.pitch = 0.0f;
        e.yaw   = 0.0f;
        return e;
    }

    return sys->relEuler;
}

AxisAngleDeg_t DualIMU_GetRelativeAxisAngle(const DualIMU_t *sys)
{
    AxisAngleDeg_t aa;

    if (sys == 0)
    {
        aa.axis = Vec3f_Create(0.0f, 0.0f, 0.0f);
        aa.angle_deg = 0.0f;
        return aa;
    }

    return sys->relAxisAngle;
}
