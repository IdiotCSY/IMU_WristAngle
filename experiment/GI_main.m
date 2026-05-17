%% GI_main.m
% 双 IMU 纯陀螺积分 GI 主脚本
%
% COM5 = hand
% COM6 = arm
%
% 依赖 LIBRARY 中的函数：
%   wit_parse_packet.m
%   wit_parse_stream.m
%   wit_read_gyro_acc.m
%   gyro_integrate.m
%   quat_mul.m
%   quat_conj.m
%   quat_normalize.m
%   quat_to_eulZYX_deg.m
%   quat_to_axis_angle_deg.m
%   imu_realtime_plot_init.m
%   imu_realtime_plot_update.m
%   imu_realtime_plot_reset.m

clear; clc; close all;
clear sHand sArm;

%% 0. 路径设置

if exist('setup_project_paths', 'file') == 2
    setup_project_paths();
else
    if isfolder('LIBRARY')
        addpath('LIBRARY');
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

%% 2. GI 参数

Fs = 200;          % 当前按上位机 200 Hz 输出设置
Ts = 1 / Fs;

% 注意：前面实测静止 GYRO 全为 0，说明模块已做内部零偏处理。
% 因此这里不再做外部 bias 扣除，避免过补偿。
biasHand_deg_s = [0 0 0];
biasArm_deg_s  = [0 0 0];

%% 3. 姿态初始化

qHand = [1 0 0 0];
qArm  = [1 0 0 0];

qRel0 = [1 0 0 0];

%% 4. 绘图初始化

plotParams = struct();
plotParams.targetFPS = 25;
plotParams.showWindow = 8;
plotParams.maxPoints = 500;

% % 放大观察轴角小偏差
% plotParams.thetaYLim = [0 30];
% plotParams.thetaYTick = 0:5:30;
plotParams.thetaYLim = [0 180];
plotParams.thetaYTick = 0:30:180;

plotParams.eulerYLim = [-180 180];
plotParams.eulerYTick = -180:60:180;

P = imu_realtime_plot_init(plotParams);

disp("GI 主脚本启动。");
disp("COM5 = hand，COM6 = arm。");
disp("按 z 可重新置零。关闭图窗停止。");

tStart = tic;

%% 5. 主循环

while ishandle(P.fig)

    %% 5.1 读取 hand / arm 角速度

    [gyroHandList, ~, bufferHand] = wit_read_gyro_acc(sHand, bufferHand);
    [gyroArmList,  ~, bufferArm]  = wit_read_gyro_acc(sArm,  bufferArm);

    %% 5.2 hand 姿态积分

    for i = 1:size(gyroHandList, 1)
        omega_deg_s = gyroHandList(i,:) - biasHand_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;

        qHand = gyro_integrate(qHand, omega_rad_s, Ts);
    end

    %% 5.3 arm 姿态积分

    for i = 1:size(gyroArmList, 1)
        omega_deg_s = gyroArmList(i,:) - biasArm_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;

        qArm = gyro_integrate(qArm, omega_rad_s, Ts);
    end

    %% 5.4 按 z 重新置零

    if P.fig.UserData.zeroRequested
        qHand = [1 0 0 0];
        qArm  = [1 0 0 0];
        qRel0 = [1 0 0 0];

        P = imu_realtime_plot_reset(P);
        tStart = tic;

        disp("GI 已重新置零：qHand、qArm、qRel0 重置为单位四元数。");
    end

    %% 5.5 计算相对姿态

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = quat_normalize(qRel);

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = quat_normalize(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    %% 5.6 更新显示

    tNow = toc(tStart);

    qHandShow = qHand;
    qArmShow  = qArm;

    P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow);

    pause(0.001);
end

%% 6. 释放串口

clear sHand sArm;
disp("程序结束，串口已释放。");