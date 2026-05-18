%% raw_main.m
% 双 IMU 原厂内置四元数对比主脚本
%
% COM5 = hand
% COM6 = arm
%
% 说明：
%   本脚本不使用 GI / MH 自己解算算法。
%   它直接读取 WT-IMU63 原厂内部输出的四元数 q0~q3，
%   然后计算 hand 相对于 arm 的相对姿态：
%
%       qRel = qArm^{-1} ⊗ qHand
%
%   适合用于和 GI_main.m、MH_main.m 做对比。
%
% 上位机设置：
%   输出内容：四元数
%   通讯速率：115200
%   回传速率：50 Hz 或 200 Hz
%
% 按键：
%   z : 将当前相对姿态定义为零位

clear; clc; close all;
clear sHand sArm;

%% 0. 路径设置

if exist('setup_project_paths', 'file') == 2
    setup_project_paths();
else
    if isfolder('LIBRARY')
        addpath('LIBRARY');
    elseif isfolder(fullfile('experiment','LIBRARY'))
        addpath(fullfile('experiment','LIBRARY'));
    end
end

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

qHand = [];
qArm  = [];

%% 2. 零位变量

% 相对姿态零位
qRel0 = [1 0 0 0];

% 3D 显示零位
qHand0 = [];
qArm0  = [];

%% 3. 绘图初始化

plotParams = struct();
plotParams.targetFPS = 25;
plotParams.showWindow = 8;
plotParams.maxPoints = 500;

% 为了和 MH_validate_points 统一，这里轴角显示 0~180 deg
plotParams.thetaYLim = [0 180];
plotParams.thetaYTick = 0:30:180;

plotParams.eulerYLim = [-180 180];
plotParams.eulerYTick = -180:60:180;

P = imu_realtime_plot_init(plotParams);

disp("原厂四元数 raw_main 启动。");
disp("COM5 = hand，COM6 = arm。");
disp("当前脚本直接读取 WT-IMU63 原厂输出四元数。");
disp("按 z：将当前相对姿态定义为零位。关闭图窗停止。");

tStart = tic;

%% 4. 主循环

while ishandle(P.fig)

    %% 4.1 读取原厂四元数

    [qNewHand, bufferHand] = wit_read_quat_latest(sHand, bufferHand);
    [qNewArm,  bufferArm]  = wit_read_quat_latest(sArm,  bufferArm);

    if ~isempty(qNewHand)
        qHand = qNewHand;
    end

    if ~isempty(qNewArm)
        qArm = qNewArm;
    end

    % 如果还没有同时读到两个 IMU 的四元数，先等待
    if isempty(qHand) || isempty(qArm)
        pause(0.001);
        continue;
    end

    %% 4.2 当前相对姿态

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = quat_normalize(qRel);

    %% 4.3 按 z：记录当前姿态为零位

    if P.fig.UserData.zeroRequested

        qHand0 = qHand;
        qArm0  = qArm;
        qRel0  = qRel;

        P = imu_realtime_plot_reset(P);
        P.fig.UserData.zeroRequested = false;

        tStart = tic;

        disp("raw_main 已重新定义当前相对姿态为零位。");
    end

    %% 4.4 零位补偿后的相对姿态

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = quat_normalize(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    %% 4.5 3D 显示姿态

    if isempty(qHand0)
        qHandShow = qHand;
    else
        qHandShow = quat_mul(quat_conj(qHand0), qHand);
        qHandShow = quat_normalize(qHandShow);
    end

    if isempty(qArm0)
        qArmShow = qArm;
    else
        qArmShow = quat_mul(quat_conj(qArm0), qArm);
        qArmShow = quat_normalize(qArmShow);
    end

    %% 4.6 更新显示

    tNow = toc(tStart);

    P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow);

    pause(0.001);
end

%% 5. 释放串口

clear sHand sArm;
disp("程序结束，串口已释放。");

%% ================= 局部函数 =================

function [qLatest, buffer] = wit_read_quat_latest(s, buffer)
%WIT_READ_QUAT_LATEST 读取 WT-IMU63 最新一个四元数包
%
% 输入：
%   s      : serialport 串口对象
%   buffer : 上一轮未解析完的残余字节
%
% 输出：
%   qLatest : 最新四元数 [w x y z]；如果本轮没读到四元数，则为空 []
%   buffer  : 本轮解析后的残余字节
%
% 依赖：
%   wit_parse_stream.m

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
            qLatest = quat_normalize(q);
        end
    end
end

end