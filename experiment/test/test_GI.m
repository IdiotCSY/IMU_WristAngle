%% test_GI.m
% COM5 = hand
% COM6 = arm
% 使用原始角速度做 Gyro Integration，不使用模块内置四元数
%
% 显示：
% 左上：Hand IMU GI姿态
% 右上：Arm  IMU GI姿态
% 左下：相对欧拉角 roll / pitch / yaw
% 右下：轴角总旋转角 theta
%
% 按 z：重新置零
%
% 上位机建议：
% 1. 只输出角速度
% 2. baud = 115200
% 3. 回传速率 = 200Hz
% 4. 带宽 = 188Hz

clear; clc; close all;
clear sHand sArm;

%% 1. 串口设置

comHand = "COM5";
comArm  = "COM6";
baud = 115200;

sHand = serialport(comHand, baud, "Timeout", 0.05);
sArm  = serialport(comArm,  baud, "Timeout", 0.05);

flush(sHand);
flush(sArm);

bufferHand = uint8([]);
bufferArm  = uint8([]);

%% 2. 参数

Fs = 200;
Ts = 1 / Fs;

% calibTime = 10.0;      % 静止标定3秒
targetFPS = 25;       % 图像刷新率
drawInterval = 1 / targetFPS;

showWindow = 8;
maxPoints = 500;

% %% 3. 静止标定陀螺零偏
% 
% disp("保持两个 IMU 静止，开始估计陀螺零偏...");
% 
% gyroHandAll = [];
% gyroArmAll  = [];
% 
% t0 = tic;
% 
% while toc(t0) < calibTime
%     [gyroHandList, bufferHand] = read_gyro_packets(sHand, bufferHand);
%     [gyroArmList,  bufferArm]  = read_gyro_packets(sArm,  bufferArm);
% 
%     if ~isempty(gyroHandList)
%         gyroHandAll = [gyroHandAll; gyroHandList]; %#ok<AGROW>
%     end
% 
%     if ~isempty(gyroArmList)
%         gyroArmAll = [gyroArmAll; gyroArmList]; %#ok<AGROW>
%     end
% 
%     pause(0.001);
% end
% 
% if isempty(gyroHandAll) || isempty(gyroArmAll)
%     error("没有读到足够的角速度数据，请检查是否只勾选了角速度输出。");
% end
% 
% biasHand_deg_s = mean(gyroHandAll, 1);
% biasArm_deg_s  = mean(gyroArmAll, 1);
% 
% fprintf("Hand gyro bias = %.4f %.4f %.4f deg/s\n", biasHand_deg_s);
% fprintf("Arm  gyro bias = %.4f %.4f %.4f deg/s\n", biasArm_deg_s);
% 
% disp("零偏标定完成，开始 GI 实时积分。");

biasHand_deg_s = [0 0 0];
biasArm_deg_s  = [0 0 0];

%% 4. 初始化姿态

qHand = [1 0 0 0];
qArm  = [1 0 0 0];

qRel0 = [1 0 0 0];

%% 5. 创建单窗口显示

fig = figure('Name','Two IMU GI Realtime Display', ...
    'Color','w', ...
    'NumberTitle','off');

fig.UserData.zeroRequested = false;
set(fig, 'KeyPressFcn', @key_press_callback);

tl = tiledlayout(fig, 2, 2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

% 左上：Hand 3D
axHand = nexttile(tl, 1);
tHand = create_imu_axes(axHand, "Hand IMU GI Attitude - COM5");

% 右上：Arm 3D
axArm = nexttile(tl, 2);
tArm = create_imu_axes(axArm, "Arm IMU GI Attitude - COM6");

% 左下：相对欧拉角
axEuler = nexttile(tl, 3);
hold(axEuler, 'on'); grid(axEuler, 'on');

hRoll  = plot(axEuler, nan, nan, 'r', 'LineWidth', 1.4);
hPitch = plot(axEuler, nan, nan, 'g', 'LineWidth', 1.4);
hYaw   = plot(axEuler, nan, nan, 'b', 'LineWidth', 1.4);

xlabel(axEuler, 'Time (s)');
ylabel(axEuler, 'Relative Euler angle (deg)');
legend(axEuler, 'Roll_{rel}', 'Pitch_{rel}', 'Yaw_{rel}', 'Location','best');
title(axEuler, 'GI Relative Euler Angles');
ylim(axEuler, [-180 180]);
yticks(axEuler, -180:60:180);

% 右下：轴角 theta
axTheta = nexttile(tl, 4);
hold(axTheta, 'on');
grid(axTheta, 'on');

hTheta = plot(axTheta, nan, nan, 'k', 'LineWidth', 1.6);

xlabel(axTheta, 'Time (s)');
ylabel(axTheta, '\theta (deg)');
title(axTheta, 'GI Relative Axis-Angle Rotation');

% 放大观察小偏差
thetaYLim = [0 180];
ylim(axTheta, thetaYLim);
yticks(axTheta, 0:30:180);

%% 6. 实时积分与显示

tLog = [];
eulLog = [];
thetaLog = [];

tStart = tic;
lastDraw = tic;
lastXlimUpdate = tic;

disp("开始运行。按 z 可重新置零，关闭窗口停止。");

while ishandle(fig)

    %% 6.1 读取并积分 hand

    [gyroHandList, bufferHand] = read_gyro_packets(sHand, bufferHand);

    for i = 1:size(gyroHandList, 1)
        omega_deg_s = gyroHandList(i,:) - biasHand_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;
        qHand = gyro_integrate(qHand, omega_rad_s, Ts);
    end

    %% 6.2 读取并积分 arm

    [gyroArmList, bufferArm] = read_gyro_packets(sArm, bufferArm);

    for i = 1:size(gyroArmList, 1)
        omega_deg_s = gyroArmList(i,:) - biasArm_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;
        qArm = gyro_integrate(qArm, omega_rad_s, Ts);
    end

    %% 6.3 按 z 重新置零

    if fig.UserData.zeroRequested
        qHand = [1 0 0 0];
        qArm  = [1 0 0 0];
        qRel0 = [1 0 0 0];

        tLog = [];
        eulLog = [];
        thetaLog = [];

        set(hRoll,  'XData', nan, 'YData', nan);
        set(hPitch, 'XData', nan, 'YData', nan);
        set(hYaw,   'XData', nan, 'YData', nan);
        set(hTheta, 'XData', nan, 'YData', nan);

        tStart = tic;
        fig.UserData.zeroRequested = false;

        disp("已重新置零：qHand、qArm、qRel0 均重置。");
    end

    %% 6.4 限制显示刷新率

    if toc(lastDraw) < drawInterval
        pause(0.001);
        continue;
    end

    tNow = toc(tStart);

    %% 6.5 更新两个 IMU 的 3D 姿态

    RHand = quat_to_rotm_scalar_first(qHand);
    RArm  = quat_to_rotm_scalar_first(qArm);

    THand = eye(4);
    THand(1:3,1:3) = RHand;
    set(tHand, 'Matrix', THand);

    TArm = eye(4);
    TArm(1:3,1:3) = RArm;
    set(tArm, 'Matrix', TArm);

    %% 6.6 相对姿态 q_rel = q_arm^{-1} ⊗ q_hand

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = qRel / norm(qRel);

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = qOut / norm(qOut);

    eul = quat_to_eulZYX_deg(qOut);
    eul = wrap_to_180(eul);

    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);
    thetaDeg = min(max(thetaDeg, 0), 180);

    %% 6.7 记录数据

    tLog = [tLog; tNow];
    eulLog = [eulLog; eul];
    thetaLog = [thetaLog; thetaDeg];

    if numel(tLog) > maxPoints
        tLog = tLog(end-maxPoints+1:end);
        eulLog = eulLog(end-maxPoints+1:end,:);
        thetaLog = thetaLog(end-maxPoints+1:end);
    end

    %% 6.8 更新图像

    set(hRoll,  'XData', tLog, 'YData', eulLog(:,1));
    set(hPitch, 'XData', tLog, 'YData', eulLog(:,2));
    set(hYaw,   'XData', tLog, 'YData', eulLog(:,3));
    set(hTheta, 'XData', tLog, 'YData', thetaLog);

    if toc(lastXlimUpdate) > 0.4
        xStart = max(0, tNow - showWindow);
        xEnd = max(showWindow, tNow);

        xlim(axEuler, [xStart xEnd]);
        xlim(axTheta, [xStart xEnd]);

        ylim(axEuler, [-180 180]);
        ylim(axTheta, thetaYLim);

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
    disp("收到按键 z：准备重新置零。");
end

end

function [gyroList, buffer] = read_gyro_packets(s, buffer)
% 返回本轮读到的所有角速度包，单位 deg/s
% gyroList: N x 3

gyroList = [];

n = s.NumBytesAvailable;
if n <= 0
    return;
end

newBytes = read(s, n, "uint8");
[outs, buffer] = wit_parse_stream(buffer, newBytes);

for k = 1:numel(outs)
    out = outs{k};

    if strcmp(out.typeName, 'GYRO')
        gyroList = [gyroList; out.gyro_deg_s]; %#ok<AGROW>
    end
end

end

function qNew = gyro_integrate(q, omega_rad_s, Ts)
% q: [w x y z]
% omega_rad_s: 1x3, body frame angular velocity, rad/s

q = q(:).';
omega = omega_rad_s(:).';

theta = norm(omega) * Ts;

if theta < 1e-12
    dq = [1, 0.5 * omega * Ts];
else
    axis = omega / norm(omega);
    dq = [cos(theta/2), axis * sin(theta/2)];
end

qNew = quat_mul(q, dq);
qNew = qNew / norm(qNew);

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

function qc = quat_conj(q)

q = q(:).';
qc = [q(1), -q(2), -q(3), -q(4)];

end

function q = quat_mul(q1, q2)

q1 = q1(:).';
q2 = q2(:).';

s1 = q1(1);
v1 = q1(2:4);

s2 = q2(1);
v2 = q2(2:4);

s = s1*s2 - dot(v1,v2);
v = s1*v2 + s2*v1 + cross(v1,v2);

q = [s, v];

end

function R = quat_to_rotm_scalar_first(q)

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

if q(1) < 0
    q = -q;
end

w = q(1);
v = q(2:4);

vNorm = norm(v);
angle = 2 * atan2(vNorm, w);

if vNorm < 1e-8
    axis = [0 0 0];
    angleDeg = 0;
else
    axis = v / vNorm;
    angleDeg = angle * 180/pi;
end

end

function angleWrapped = wrap_to_180(angleDeg)

angleWrapped = mod(angleDeg + 180, 360) - 180;

end