function qNew = gyro_integrate(q, omega_rad_s, Ts)
%GYRO_INTEGRATE 使用角速度对四元数进行一步积分
%
% 输入：
%   q           : 当前姿态四元数 [w x y z]
%   omega_rad_s : 角速度 [wx wy wz]，单位 rad/s，body 坐标系
%   Ts          : 采样周期，单位 s
%
% 输出：
%   qNew : 更新后的姿态四元数 [w x y z]
%
% 注意：
%   该函数只做纯陀螺积分，不包含加速度计修正。

q = quat_normalize(q);
omega = omega_rad_s(:).';

theta = norm(omega) * Ts;

if theta < 1e-12
    dq = [1, 0.5 * omega * Ts];
else
    axis = omega / norm(omega);
    dq = [cos(theta/2), axis * sin(theta/2)];
end

qNew = quat_mul(q, dq);
qNew = quat_normalize(qNew);
end