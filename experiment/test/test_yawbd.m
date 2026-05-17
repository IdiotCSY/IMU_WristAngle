%% test_yawbd.m
% COM5 = hand
% COM6 = arm
%
% 单窗口显示：
% 左上：Hand IMU 3D姿态
% 右上：Arm  IMU 3D姿态
% 左下：相对欧拉角 roll / pitch / yaw，范围固定 [-180, 180]
% 右下：轴角总旋转角 theta，范围固定 [0, 180]
%
% 按 z：记录当前姿态为零位
%
% 前提：
% 1. 两个 WT-IMU63 均设置为 115200 波特率
% 2. 建议输出内容只勾选四元数
% 3. 已有 wit_parse_stream.m 和 wit_parse_packet.m

clear; clc; close all;
clear sHand sArm;

%% 1. 串口设置

comHand = "COM5";   % hand IMU
comArm  = "COM6";   % arm IMU
baud = 115200;

sHand = serialport(comHand, baud, "Timeout", 0.05);
sArm  = serialport(comArm,  baud, "Timeout", 0.05);

flush(sHand);
flush(sArm);

bufferHand = uint8([]);
bufferArm  = uint8([]);

qHandLatest = [];
qArmLatest  = [];

% 用于 3D 显示零位
qHand0 = [];
qArm0  = [];

% 用于相对姿态零位
qRel0 = [];

%% 2. 显示参数

targetFPS = 25;
drawInterval = 1 / targetFPS;

showWindow = 8;
maxPoints = 300;

xlimInterval = 0.4;

% 如果 3D 显示方向整体反了，把它改成 true
useTransposeFor3D = false;

%% 3. 创建单窗口

figMain = figure('Name','Two IMU Realtime Display', ...
    'Color','w', ...
    'NumberTitle','off');

figMain.UserData.zeroRequested = false;
figMain.UserData.zeroDone = false;
set(figMain, 'KeyPressFcn', @key_press_callback);

tl = tiledlayout(figMain, 2, 2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

% 左上：Hand 3D
axHand = nexttile(tl, 1);
tHand = create_imu_axes(axHand, "Hand IMU - COM5");

% 右上：Arm 3D
axArm = nexttile(tl, 2);
tArm = create_imu_axes(axArm, "Arm IMU - COM6");

% 左下：相对欧拉角
axEuler = nexttile(tl, 3);
hold(axEuler, 'on');
grid(axEuler, 'on');

hRelRoll  = plot(axEuler, nan, nan, 'r', 'LineWidth', 1.4);
hRelPitch = plot(axEuler, nan, nan, 'g', 'LineWidth', 1.4);
hRelYaw   = plot(axEuler, nan, nan, 'b', 'LineWidth', 1.4);

xlabel(axEuler, 'Time (s)');
ylabel(axEuler, 'Euler angle (deg)');
legend(axEuler, 'Roll_{rel}', 'Pitch_{rel}', 'Yaw_{rel}', 'Location','best');
title(axEuler, 'Relative Euler Angles');

ylim(axEuler, [-180 180]);
yticks(axEuler, -180:60:180);

% 右下：轴角总旋转角 theta
axTheta = nexttile(tl, 4);
hold(axTheta, 'on');
grid(axTheta, 'on');

hTheta = plot(axTheta, nan, nan, 'k', 'LineWidth', 1.6);

xlabel(axTheta, 'Time (s)');
ylabel(axTheta, '\theta (deg)');
title(axTheta, 'Axis-Angle Rotation Angle');

ylim(axTheta, [0 180]);
yticks(axTheta, 0:30:180);

%% 4. 数据缓存

tLog = [];
relEulerLog = [];
thetaLog = [];

disp("开始实时显示。COM5 = hand，COM6 = arm。");
disp("把两个 IMU 摆成同一姿态，点击图窗，然后按 z 记录零位。");
disp("关闭窗口停止程序。");

t0 = tic;
lastDraw = tic;
lastXlimUpdate = tic;

%% 5. 主循环

while ishandle(figMain)

    %% 5.1 读取最新四元数

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

    %% 5.2 当前相对姿态

    % q_rel = q_arm^{-1} ⊗ q_hand
    qRel = quat_mul(quat_conj(qArmLatest), qHandLatest);
    qRel = qRel / norm(qRel);

    %% 5.3 按 z 手动记录零位

    if figMain.UserData.zeroRequested
        qHand0 = qHandLatest;
        qArm0  = qArmLatest;
        qRel0  = qRel;

        figMain.UserData.zeroRequested = false;
        figMain.UserData.zeroDone = true;

        % 清空曲线，从零位后重新计时
        tLog = [];
        relEulerLog = [];
        thetaLog = [];

        set(hRelRoll,  'XData', nan, 'YData', nan);
        set(hRelPitch, 'XData', nan, 'YData', nan);
        set(hRelYaw,   'XData', nan, 'YData', nan);
        set(hTheta,    'XData', nan, 'YData', nan);

        t0 = tic;

        disp("已记录零位：qHand0、qArm0、qRel0。");
    end

    %% 5.4 限制刷新率

    if toc(lastDraw) < drawInterval
        pause(0.001);
        continue;
    end

    tNow = toc(t0);

    %% 5.5 更新两个 IMU 的 3D 姿态

    % 如果还没按 z，显示绝对姿态；
    % 按 z 后，显示相对于各自初始姿态的变化。
    if isempty(qHand0)
        qHandShow = qHandLatest;
    else
        qHandShow = quat_mul(quat_conj(qHand0), qHandLatest);
        qHandShow = qHandShow / norm(qHandShow);
    end

    if isempty(qArm0)
        qArmShow = qArmLatest;
    else
        qArmShow = quat_mul(quat_conj(qArm0), qArmLatest);
        qArmShow = qArmShow / norm(qArmShow);
    end

    RHand = quat_to_rotm_scalar_first(qHandShow);
    RArm  = quat_to_rotm_scalar_first(qArmShow);

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

    %% 5.6 计算零位补偿后的相对姿态

    if isempty(qRel0)
        % 还没按 z 时，先显示为零
        qOut = [1 0 0 0];
    else
        qOut = quat_mul(quat_conj(qRel0), qRel);
        qOut = qOut / norm(qOut);
    end

    relEuler = quat_to_eulZYX_deg(qOut);
    relEuler = wrap_to_180(relEuler);

    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);
    thetaDeg = min(max(thetaDeg, 0), 180);

    %% 5.7 记录数据

    tLog = [tLog; tNow];
    relEulerLog = [relEulerLog; relEuler];
    thetaLog = [thetaLog; thetaDeg];

    if numel(tLog) > maxPoints
        tLog = tLog(end-maxPoints+1:end);
        relEulerLog = relEulerLog(end-maxPoints+1:end, :);
        thetaLog = thetaLog(end-maxPoints+1:end);
    end

    %% 5.8 更新曲线

    set(hRelRoll,  'XData', tLog, 'YData', relEulerLog(:,1));
    set(hRelPitch, 'XData', tLog, 'YData', relEulerLog(:,2));
    set(hRelYaw,   'XData', tLog, 'YData', relEulerLog(:,3));
    set(hTheta,    'XData', tLog, 'YData', thetaLog);

    if toc(lastXlimUpdate) > xlimInterval
        xStart = max(0, tNow - showWindow);
        xEnd = max(showWindow, tNow);

        xlim(axEuler, [xStart, xEnd]);
        xlim(axTheta, [xStart, xEnd]);

        % 固定纵轴，防止 MATLAB 自动缩放
        ylim(axEuler, [-180 180]);
        ylim(axTheta, [0 180]);

        lastXlimUpdate = tic;
    end

    drawnow limitrate;
    lastDraw = tic;
end

clear sHand sArm;
disp("程序结束，串口已释放。");

%% ================= 局部函数 =================

function key_press_callback(src, event)

if strcmp(event.Key, 'z')
    src.UserData.zeroRequested = true;
    disp("收到按键 z：准备记录当前姿态为零位。");
end

end

function imuT = create_imu_axes(ax, titleName)

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

title(ax, titleName);

% 世界坐标轴
plot3(ax, [0 0.12], [0 0],    [0 0],    'r', 'LineWidth', 1.2);
plot3(ax, [0 0],    [0 0.12], [0 0],    'g', 'LineWidth', 1.2);
plot3(ax, [0 0],    [0 0],    [0 0.12], 'b', 'LineWidth', 1.2);

text(ax, 0.125, 0, 0, 'X', 'Color','r');
text(ax, 0, 0.125, 0, 'Y', 'Color','g');
text(ax, 0, 0, 0.125, 'Z', 'Color','b');

imuT = hgtransform('Parent', ax);

% IMU板子模型
[V,F] = make_box([0.09, 0.06, 0.012]);

patch('Vertices', V, 'Faces', F, ...
    'FaceColor', [0.75 0.82 1.0], ...
    'FaceAlpha', 0.9, ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'Parent', imuT);

% IMU自身坐标轴
L = 0.09;

line([0 L], [0 0], [0 0], 'Color','r', 'LineWidth',2.2, 'Parent',imuT);
line([0 0], [0 L], [0 0], 'Color','g', 'LineWidth',2.2, 'Parent',imuT);
line([0 0], [0 0], [0 L], 'Color','b', 'LineWidth',2.2, 'Parent',imuT);

text(L, 0, 0, 'x', 'Parent', imuT, 'Color','r');
text(0, L, 0, 'y', 'Parent', imuT, 'Color','g');
text(0, 0, L, 'z', 'Parent', imuT, 'Color','b');

end

function [V,F] = make_box(dim)
% 生成中心在原点的长方体
% dim = [Lx Ly Lz]

Lx = dim(1);
Ly = dim(2);
Lz = dim(3);

x = Lx/2;
y = Ly/2;
z = Lz/2;

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
% 从串口对象中读取最新四元数
% 本轮没有读到四元数时返回 []

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
% q = [w x y z]

q = q(:).';
qc = [q(1), -q(2), -q(3), -q(4)];

end

function q = quat_mul(q1, q2)
% q = q1 ⊗ q2
% 标量在前：[w x y z]

q1 = q1(:).';
q2 = q2(:).';

s1 = q1(1);
v1 = q1(2:4);

s2 = q2(1);
v2 = q2(2:4);

s = s1*s2 - dot(v1, v2);
v = s1*v2 + s2*v1 + cross(v1, v2);

q = [s, v];

end

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

function eulDeg = quat_to_eulZYX_deg(q)
% 输出 [roll pitch yaw]，单位 deg
% ZYX顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)

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
% 四元数转轴角
% axis: 单位旋转轴
% angleDeg: 总旋转角，单位 deg

q = q(:).' / norm(q);

% q 和 -q 表示同一姿态，统一符号，避免角度跳变
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

function angleWrapped = wrap_to_180(angleDeg)
% 把角度限制到 [-180, 180]

angleWrapped = mod(angleDeg + 180, 360) - 180;

end