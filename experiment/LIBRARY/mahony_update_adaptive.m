function [qNew, intErrNew, info] = mahony_update_adaptive(q, intErr, gyro_rad_s, acc_mps2, Ts, params)
%MAHONY_UPDATE_ADAPTIVE 自适应 Mahony 姿态更新
%
% 功能：
%   在纯陀螺积分 GI 的基础上，用加速度计在低动态时修正 roll/pitch。
%
% 输入：
%   q          : 当前姿态四元数 [w x y z]，body -> world
%   intErr     : 积分误差项 [ex ey ez]
%   gyro_rad_s : 角速度 [wx wy wz]，单位 rad/s，body 坐标系
%   acc_mps2   : 加速度 [ax ay az]，单位 m/s^2，body 坐标系
%   Ts         : 采样周期，单位 s
%   params     : 参数结构体
%
% 输出：
%   qNew       : 更新后的姿态四元数 [w x y z]
%   intErrNew  : 更新后的积分误差项
%   info       : 调试信息，包括加速度权重、误差向量等
%
% 注意：
%   1. 该算法主要修正 roll/pitch，不能根治 yaw 漂移。
%   2. 快速运动时加速度计包含动态加速度，因此会自动降低加速度权重。
%   3. 建议第一版 Ki = 0，先只用比例修正。

q = quat_normalize(q);
intErr = intErr(:).';
gyro_rad_s = gyro_rad_s(:).';
acc_mps2 = acc_mps2(:).';

if ~isfield(params, 'Kp'); params.Kp = 0.5; end
if ~isfield(params, 'Ki'); params.Ki = 0.0; end
if ~isfield(params, 'g'); params.g = 9.81; end
if ~isfield(params, 'accTol'); params.accTol = 1.0; end          % m/s^2
if ~isfield(params, 'gyroTolDeg'); params.gyroTolDeg = 100; end  % deg/s
if ~isfield(params, 'intLimit'); params.intLimit = 0.5; end

info = struct();
info.accWeight = 0;
info.e = [0 0 0];
info.accNorm = norm(acc_mps2);

%% 1. 判断加速度计可信度

accNorm = norm(acc_mps2);

if accNorm < 1e-8
    accWeight = 0;
    e = [0 0 0];
else
    accUnit = acc_mps2 / accNorm;

    % 加速度模长越接近 g，越可信
    accErr = abs(accNorm - params.g);
    % wAccNorm = 1 - accErr / params.accTol;
    % wAccNorm = min(max(wAccNorm, 0), 1);
    % 
    % % 角速度越小，越适合用加速度修正
    % gyroNormDeg = norm(gyro_rad_s) * 180/pi;
    % wGyro = 1 - gyroNormDeg / params.gyroTolDeg;
    % wGyro = min(max(wGyro, 0), 1);
    % 
    % accWeight = wAccNorm * wGyro;
    gyroNormDeg = norm(gyro_rad_s) * 180/pi;

    if accErr > params.accTol || gyroNormDeg > params.gyroTolDeg
        accWeight = 0;
    else
        wAccNorm = 1 - accErr / params.accTol;
        wGyro = 1 - gyroNormDeg / params.gyroTolDeg;
    
        wAccNorm = min(max(wAccNorm, 0), 1);
        wGyro = min(max(wGyro, 0), 1);
    
        % 平方后更保守，避免动态加速度轻易介入
        accWeight = (wAccNorm * wGyro)^2;
    end
   

    %% 2. 计算预测重力方向

    R = quat_to_rotm_scalar_first(q);

    % 世界系重力方向，归一化
    gWorld = [0; 0; 1];

    % 预测的重力方向在 body 坐标系下的表达
    gHatBody = R.' * gWorld;
    gHatBody = gHatBody(:).';

    % 姿态误差：测得重力方向 vs 预测重力方向
    % 这里用 cross(accUnit, gHatBody)，和 q = q ⊗ dq 的积分方向匹配
    e = cross(accUnit, gHatBody);
end

%% 3. PI 修正角速度

intErrNew = intErr + accWeight * e * Ts;

% 防止积分项过大
intErrNew = min(max(intErrNew, -params.intLimit), params.intLimit);

omegaCorr = gyro_rad_s + params.Kp * accWeight * e + params.Ki * intErrNew;

%% 4. 用修正后的角速度积分

qNew = gyro_integrate(q, omegaCorr, Ts);
qNew = quat_normalize(qNew);

%% 5. 调试信息

info.accWeight = accWeight;
info.e = e;
info.accNorm = accNorm;

end

function R = quat_to_rotm_scalar_first(q)
% q = [w x y z]

q = quat_normalize(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R = zeros(3,3);

R(1,1) = 1 - 2*(y^2 + z^2);
R(1,2) = 2*(x*y - w*z);
R(1,3) = 2*(x*z + w*y);

R(2,1) = 2*(x*y + w*z);
R(2,2) = 1 - 2*(x^2 + z^2);
R(2,3) = 2*(y*z - w*x);

R(3,1) = 2*(x*z - w*y);
R(3,2) = 2*(y*z + w*x);
R(3,3) = 1 - 2*(x^2 + y^2);

end