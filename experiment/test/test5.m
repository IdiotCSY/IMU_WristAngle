%% two_imu_realtime_fast_v2.m
% COM5 = hand
% COM6 = arm
% 显示：
% 1. hand IMU 3D姿态窗口
% 2. arm  IMU 3D姿态窗口
% 3. 相对姿态窗口：
%    - 相对欧拉角 roll/pitch/yaw
%    - 轴角总旋转角 theta
%
% 建议上位机只勾选四元数，波特率 115200，回传速率 50Hz

clear; clc; close all;
clear sHand sArm;

%% 串口设置

comHand = "COM5";
comArm  = "COM6";
baud = 115200;

sHand = serialport(comHand, baud, "Timeout", 0.05);
sArm  = serialport(comArm,  baud, "Timeout", 0.05);

flush(sHand);
flush(sArm);

bufferHand = uint8([]);
bufferArm  = uint8([]);

qHandLatest = [];
qArmLatest  = [];
qRel0 = [];

%% 显示参数

targetFPS = 30;
drawInterval = 1 / targetFPS;

showWindow = 10;   % 曲线显示最近10秒
maxPoints = 500;   % 最多保存500个显示点

% 如果3D转动方向看起来反了，改成 true 试试
useTransposeFor3D = false;

%% 3D窗口：hand

[figHand, tHand] = create_imu_figure("Hand IMU attitude - COM5");

%% 3D窗口：arm

[figArm, tArm] = create_imu_figure("Arm IMU attitude - COM6");

%% 相对姿态曲线窗口

figRel = figure('Name','Relative Attitude: hand w.r.t. arm','Color','w');
tl = tiledlayout(figRel, 2, 1);

axEuler = nexttile(tl);
hold(axEuler, 'on'); grid(axEuler, 'on');
hRelRoll  = plot(axEuler, nan, nan, 'r', 'LineWidth', 1.4);
hRelPitch = plot(axEuler, nan, nan, 'g', 'LineWidth', 1.4);
hRelYaw   = plot(axEuler, nan, nan, 'b', 'LineWidth', 1.4);
ylabel(axEuler, 'Euler angle (deg)');
legend(axEuler, 'Roll_{rel}', 'Pitch_{rel}', 'Yaw_{rel}');
title(axEuler, 'Relative Euler Angles');

axTheta = nexttile(tl);
hold(axTheta, 'on'); grid(axTheta, 'on');
hTheta = plot(axTheta, nan, nan, 'k', 'LineWidth', 1.6);
xlabel(axTheta, 'Time (s)');
ylabel(axTheta, '\theta (deg)');
title(axTheta, 'Relative Axis-Angle: Rotation Angle');

%% 数据缓存

tLog = [];
relEulerLog = [];
thetaLog = [];

disp("开始实时显示。COM5=hand，COM6=arm。");
disp("启动时保持两个IMU静止，程序会自动记录初始相对零位。");
disp("关闭任意窗口停止。");

t0 = tic;
lastDraw = tic;

%% 主循环

while ishandle(figHand) && ishandle(figArm) && ishandle(figRel)

    [qNewHand, bufferHand] = read_latest_quat_from_imu(sHand, bufferHand);
    [qNewArm,  bufferArm]  = read_latest_quat_from_imu(sArm,  bufferArm);

    if ~isempty(qNewHand)
        qHandLatest = qNewHand;
    end

    if ~isempty(qNewArm)
        qArmLatest = qNewArm;
    end

    if isempty(qHandLatest) || isempty(qArmLatest)
        pause(0.001);
        continue;
    end

    if toc(lastDraw) < drawInterval
        pause(0.001);
        continue;
    end

    tNow = toc(t0);

    %% 1. 更新两个IMU的3D姿态

    RHand = quat_to_rotm_scalar_first(qHandLatest);
    RArm  = quat_to_rotm_scalar_first(qArmLatest);

    if useTransposeFor3D
        RHand = RHand.';
        RArm  = RArm.';
    end

    THand = eye(4);
    THand(1:3,1:3) = RHand;
    set(tHand, 'Matrix', THand);

    TArm = eye(4);
    TArm(1:3,1:3) = RArm;
    set(tArm, 'Matrix', TArm);

    %% 2. hand 相对于 arm 的相对姿态

    qRel = quat_mul(quat_conj(qArmLatest), qHandLatest);
    qRel = qRel / norm(qRel);

    if isempty(qRel0)
        qRel0 = qRel;
        disp("已记录初始相对零位 qRel0。");
    end

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = qOut / norm(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    %% 3. 记录并裁剪数据

    tLog = [tLog; tNow];
    relEulerLog = [relEulerLog; relEuler];
    thetaLog = [thetaLog; thetaDeg];

    if numel(tLog) > maxPoints
        tLog = tLog(end-maxPoints+1:end);
        relEulerLog = relEulerLog(end-maxPoints+1:end, :);
        thetaLog = thetaLog(end-maxPoints+1:end);
    end

    %% 4. 更新曲线

    set(hRelRoll,  'XData', tLog, 'YData', relEulerLog(:,1));
    set(hRelPitch, 'XData', tLog, 'YData', relEulerLog(:,2));
    set(hRelYaw,   'XData', tLog, 'YData', relEulerLog(:,3));

    set(hTheta, 'XData', tLog, 'YData', thetaLog);

    xStart = max(0, tNow - showWindow);
    xEnd = max(showWindow, tNow);

    xlim(axEuler, [xStart, xEnd]);
    xlim(axTheta, [xStart, xEnd]);

    drawnow limitrate nocallbacks;
    lastDraw = tic;
end

clear sHand sArm;
disp("程序结束，串口已释放。");

%% ================= 局部函数 =================

function [fig, imuT] = create_imu_figure(figName)

fig = figure('Name', figName, 'Color', 'w');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 3);

xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');

xlim(ax, [-0.15 0.15]);
ylim(ax, [-0.15 0.15]);
zlim(ax, [-0.15 0.15]);

title(ax, figName);

% 世界坐标轴
plot3(ax, [0 0.12], [0 0],    [0 0],    'r', 'LineWidth', 1.5);
plot3(ax, [0 0],    [0 0.12], [0 0],    'g', 'LineWidth', 1.5);
plot3(ax, [0 0],    [0 0],    [0 0.12], 'b', 'LineWidth', 1.5);
text(ax, 0.125, 0, 0, 'X');
text(ax, 0, 0.125, 0, 'Y');
text(ax, 0, 0, 0.125, 'Z');

imuT = hgtransform('Parent', ax);

% IMU板子
[V,F] = make_box([0.09, 0.06, 0.012]);
patch('Vertices', V, 'Faces', F, ...
    'FaceColor', [0.75 0.82 1.0], ...
    'FaceAlpha', 0.9, ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'Parent', imuT);

% IMU自身坐标轴
L = 0.09;
line([0 L], [0 0], [0 0], 'Color','r', 'LineWidth',2.5, 'Parent',imuT);
line([0 0], [0 L], [0 0], 'Color','g', 'LineWidth',2.5, 'Parent',imuT);
line([0 0], [0 0], [0 L], 'Color','b', 'LineWidth',2.5, 'Parent',imuT);

text(L, 0, 0, 'x', 'Parent', imuT, 'Color', 'r');
text(0, L, 0, 'y', 'Parent', imuT, 'Color', 'g');
text(0, 0, L, 'z', 'Parent', imuT, 'Color', 'b');

end

function [V,F] = make_box(dim)
% 中心在原点的长方体

Lx = dim(1); Ly = dim(2); Lz = dim(3);
x = Lx/2; y = Ly/2; z = Lz/2;

V = [
    -x -y -z
     x -y -z
     x  y -z
    -x  y -z
    -x -y  z
     x -y  z
     x  y  z
    -x  y  z
];

F = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
];
end

function [qLatest, buffer] = read_latest_quat_from_imu(s, buffer)

qLatest = [];

n = s.NumBytesAvailable;

if n <= 0
    return;
end

newBytes = read(s, n, "uint8");
[outs, buffer] = wit_parse_stream(buffer, newBytes);

for k = 1:numel(outs)
    out = outs{k};

    if strcmp(out.typeName, 'QUAT')
        q = out.quat;
        if norm(q) > 1e-8
            qLatest = q / norm(q);
        end
    end
end

end

function qc = quat_conj(q)
q = q(:).';
qc = [q(1), -q(2), -q(3), -q(4)];
end

function q = quat_mul(q1, q2)
q1 = q1(:).';
q2 = q2(:).';

s1 = q1(1); v1 = q1(2:4);
s2 = q2(1); v2 = q2(2:4);

s = s1*s2 - dot(v1,v2);
v = s1*v2 + s2*v1 + cross(v1,v2);

q = [s, v];
end

function R = quat_to_rotm_scalar_first(q)
% q = [w x y z]

q = q(:) / norm(q);

w = q(1); x = q(2); y = q(3); z = q(4);

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

function eulDeg = quat_to_eulZYX_deg(q)
% 输出 [roll pitch yaw]，单位 deg

q = q(:) / norm(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R11 = 1 - 2*(y^2 + z^2);
R21 = 2*(x*y + w*z);
R31 = 2*(x*z - w*y);
R32 = 2*(y*z + w*x);
R33 = 1 - 2*(x^2 + y^2);

pitch = asin(max(min(-R31, 1), -1));
roll  = atan2(R32, R33);
yaw   = atan2(R21, R11);

eulDeg = [roll, pitch, yaw] * 180/pi;
end

function [axis, angleDeg] = quat_to_axis_angle_deg(q)

q = q(:).' / norm(q);

% q 和 -q 表示同一姿态，统一符号避免跳变
if q(1) < 0
    q = -q;
end

w = q(1);
v = q(2:4);

vNorm = norm(v);
angle = 2 * atan2(vNorm, w);

if vNorm < 1e-8
    axis = [0, 0, 0];
    angleDeg = 0;
else
    axis = v / vNorm;
    angleDeg = angle * 180/pi;
end

end