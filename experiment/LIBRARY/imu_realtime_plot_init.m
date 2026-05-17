function P = imu_realtime_plot_init(userParams)
%IMU_REALTIME_PLOT_INIT 初始化双IMU实时显示窗口
%
% 返回 P：绘图状态结构体
%
% 默认布局：
% 左上：Hand IMU 3D姿态
% 右上：Arm  IMU 3D姿态
% 左下：相对欧拉角 roll/pitch/yaw
% 右下：轴角 theta
%
% 按 z：设置 P.fig.UserData.zeroRequested = true
% 
% ================= 使用说明 =================
% 功能：
%   初始化双 IMU 实时显示窗口，并返回绘图状态结构体 P。
%   该函数只负责创建图窗、坐标轴、曲线对象和 3D 姿态对象，
%   不负责算法计算、不负责串口读取。
%
% 输入：
%   userParams : 可选结构体，用于覆盖默认绘图参数。
%       常用字段：
%           targetFPS       - 目标刷新帧率，例如 25
%           showWindow      - 曲线显示最近多少秒，例如 8
%           maxPoints       - 曲线最多保留多少个点，例如 500
%           thetaYLim       - 轴角纵轴范围，例如 [0 30]
%           thetaYTick      - 轴角刻度，例如 0:5:30
%           eulerYLim       - 欧拉角纵轴范围，例如 [-180 180]
%           useTransposeFor3D - 若 3D 姿态方向整体反了，可设为 true
%
% 输出：
%   P : 绘图状态结构体，后续传给 imu_realtime_plot_update 使用。
%       主要字段：
%           P.fig           - 主图窗句柄
%           P.tHand/P.tArm  - hand/arm 的 hgtransform 句柄
%           P.hRelRoll 等   - 曲线对象句柄
%           P.tLog          - 时间缓存
%           P.relEulerLog   - 相对欧拉角缓存
%           P.thetaLog      - 轴角 theta 缓存
%           P.params        - 绘图参数
%
% 注意事项：
%   1. 本函数内部绑定了键盘回调函数，按 z 后：
%          P.fig.UserData.zeroRequested = true
%      但真正的零位处理必须在主算法脚本里完成。
%
%   2. 本函数不会自动 addpath。调用前应确保 LIBRARY 已经加入 MATLAB 路径。
%
%   3. 绘图模块与算法解耦。GI、Mahony、模块自带四元数等不同算法，
%      只要最终能给出 qHandShow、qArmShow、relEuler、thetaDeg，
%      都可以共用这套绘图函数。
%
%   4. 3D 姿态显示只用于直观观察，不作为严格误差评价依据。
%      严格评价应以后处理保存的数据和误差指标为准。
%
%   5. 如果两个 IMU 的 3D 方向看起来整体反了，优先检查坐标系定义；
%      临时显示修正可以设置：
%          userParams.useTransposeFor3D = true;
% ============================================

if nargin < 1
    userParams = struct();
end

params.targetFPS = 25;
params.drawInterval = 1 / params.targetFPS;

params.showWindow = 8;
params.maxPoints = 500;

params.xlimInterval = 0.4;

params.eulerYLim = [-180 180];
params.eulerYTick = -180:60:180;

params.thetaYLim = [0 30];
params.thetaYTick = 0:5:30;

params.useTransposeFor3D = false;

params = override_params(params, userParams);

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
tHand = create_imu_axes(axHand, "Hand IMU");

% 右上：Arm 3D
axArm = nexttile(tl, 2);
tArm = create_imu_axes(axArm, "Arm IMU");

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

ylim(axEuler, params.eulerYLim);
yticks(axEuler, params.eulerYTick);

% 右下：轴角 theta
axTheta = nexttile(tl, 4);
hold(axTheta, 'on');
grid(axTheta, 'on');

hTheta = plot(axTheta, nan, nan, 'k', 'LineWidth', 1.6);

xlabel(axTheta, 'Time (s)');
ylabel(axTheta, '\theta (deg)');
title(axTheta, 'Relative Axis-Angle Rotation');

ylim(axTheta, params.thetaYLim);
yticks(axTheta, params.thetaYTick);

P = struct();

P.fig = figMain;

P.axHand = axHand;
P.axArm = axArm;
P.axEuler = axEuler;
P.axTheta = axTheta;

P.tHand = tHand;
P.tArm = tArm;

P.hRelRoll = hRelRoll;
P.hRelPitch = hRelPitch;
P.hRelYaw = hRelYaw;
P.hTheta = hTheta;

P.tLog = [];
P.relEulerLog = [];
P.thetaLog = [];

P.params = params;

P.lastDraw = tic;
P.lastXlimUpdate = tic;

end

%% ===== 局部函数 =====

function key_press_callback(src, event)

if strcmp(event.Key, 'z')
    src.UserData.zeroRequested = true;
    disp("收到按键 z：准备记录当前姿态为零位。");
end

end

function params = override_params(params, userParams)

if isempty(userParams)
    return;
end

names = fieldnames(userParams);

for i = 1:numel(names)
    params.(names{i}) = userParams.(names{i});
end

% 如果用户改了 targetFPS，同步更新 drawInterval
if isfield(userParams, 'targetFPS')
    params.drawInterval = 1 / params.targetFPS;
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

[V,F] = make_box([0.09, 0.06, 0.012]);

patch('Vertices', V, 'Faces', F, ...
    'FaceColor', [0.75 0.82 1.0], ...
    'FaceAlpha', 0.9, ...
    'EdgeColor', [0.25 0.25 0.25], ...
    'Parent', imuT);

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