function P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow)
%IMU_REALTIME_PLOT_UPDATE 更新双IMU实时显示
%
% 输入：
%   P          绘图状态结构体
%   qHandShow  hand IMU 显示姿态四元数 [w x y z]
%   qArmShow   arm  IMU 显示姿态四元数 [w x y z]
%   relEuler   相对欧拉角 [roll pitch yaw]，deg
%   thetaDeg   轴角总旋转角，deg
%   tNow       当前时间，s
% 
% ================= 使用说明 =================
% 功能：
%   根据当前算法输出结果，更新双 IMU 的 3D 姿态图和相对姿态曲线。
%   该函数只负责刷新图像，不负责串口读取、不负责姿态解算。
%
% 输入：
%   P : 绘图状态结构体，由 imu_realtime_plot_init 返回。
%
%   qHandShow : hand IMU 用于显示的姿态四元数，格式为 [w x y z]。
%       例如：
%           GI 算法中可以直接传 qHand；
%           若做了零位补偿，可以传 qHandShow = qHand0^{-1} ⊗ qHand。
%
%   qArmShow : arm IMU 用于显示的姿态四元数，格式为 [w x y z]。
%
%   relEuler : hand 相对于 arm 的相对欧拉角，单位 deg。
%       格式：
%           [roll pitch yaw]
%       函数内部会自动 wrap 到 [-180, 180]。
%
%   thetaDeg : hand 相对于 arm 的轴角总旋转角，单位 deg。
%       函数内部会取 abs(thetaDeg)，用于显示总偏差大小。
%
%   tNow : 当前时间，单位 s。
%       通常由主脚本中的：
%           tNow = toc(tStart);
%       得到。
%
% 输出：
%   P : 更新后的绘图状态结构体。
%       由于函数内部会更新 P.tLog、P.relEulerLog、P.thetaLog，
%       所以主脚本中必须写成：
%           P = imu_realtime_plot_update(...);
%
% 注意事项：
%   1. 本函数内部有限帧逻辑：
%          if toc(P.lastDraw) < P.params.drawInterval
%              return;
%          end
%      因此不是每次调用都会刷新图像，这是为了减轻 MATLAB 绘图压力。
%
%   2. 本函数会低频更新 xlim，而不是每帧更新横轴范围。
%      这样可以降低实时显示卡顿。
%
%   3. qHandShow 和 qArmShow 必须是单位四元数或接近单位四元数。
%      如果主算法中可能出现数值漂移，建议调用本函数前先做：
%          q = q / norm(q);
%
%   4. relEuler 和 thetaDeg 应由主算法根据相对四元数计算得到。
%      推荐相对姿态定义为：
%          qRel = qArm^{-1} ⊗ qHand
%
%   5. 本函数不保存实验数据。若要后处理分析，应在主脚本中另行保存
%      t、qHand、qArm、qRel、relEuler、thetaDeg 等变量。
%
%   6. 若图窗已关闭，函数会直接 return，不再更新。
% ============================================
if ~ishandle(P.fig)
    return;
end

if toc(P.lastDraw) < P.params.drawInterval
    return;
end

%% 1. 更新3D姿态

if ~isempty(qHandShow)
    RHand = quat_to_rotm_scalar_first(qHandShow);

    if P.params.useTransposeFor3D
        RHand = RHand.';
    end

    THand = eye(4);
    THand(1:3,1:3) = RHand;
    set(P.tHand, 'Matrix', THand);
end

if ~isempty(qArmShow)
    RArm = quat_to_rotm_scalar_first(qArmShow);

    if P.params.useTransposeFor3D
        RArm = RArm.';
    end

    TArm = eye(4);
    TArm(1:3,1:3) = RArm;
    set(P.tArm, 'Matrix', TArm);
end

%% 2. 更新曲线数据

relEuler = wrap_to_180(relEuler);
thetaDeg = abs(thetaDeg);

P.tLog = [P.tLog; tNow];
P.relEulerLog = [P.relEulerLog; relEuler];
P.thetaLog = [P.thetaLog; thetaDeg];

if numel(P.tLog) > P.params.maxPoints
    P.tLog = P.tLog(end-P.params.maxPoints+1:end);
    P.relEulerLog = P.relEulerLog(end-P.params.maxPoints+1:end, :);
    P.thetaLog = P.thetaLog(end-P.params.maxPoints+1:end);
end

set(P.hRelRoll,  'XData', P.tLog, 'YData', P.relEulerLog(:,1));
set(P.hRelPitch, 'XData', P.tLog, 'YData', P.relEulerLog(:,2));
set(P.hRelYaw,   'XData', P.tLog, 'YData', P.relEulerLog(:,3));
set(P.hTheta,    'XData', P.tLog, 'YData', P.thetaLog);

%% 3. 低频更新坐标轴范围

if toc(P.lastXlimUpdate) > P.params.xlimInterval
    xStart = max(0, tNow - P.params.showWindow);
    xEnd = max(P.params.showWindow, tNow);

    xlim(P.axEuler, [xStart, xEnd]);
    xlim(P.axTheta, [xStart, xEnd]);

    ylim(P.axEuler, P.params.eulerYLim);
    ylim(P.axTheta, P.params.thetaYLim);

    P.lastXlimUpdate = tic;
end

drawnow limitrate;
P.lastDraw = tic;

end

%% ===== 局部函数 =====

function R = quat_to_rotm_scalar_first(q)
% q = [w x y z]

q = q(:) / norm(q);

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

function angleWrapped = wrap_to_180(angleDeg)

angleWrapped = mod(angleDeg + 180, 360) - 180;

end