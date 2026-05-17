%% MH_main.m
% 双 IMU 自适应 Mahony / MH 主脚本
%
% COM5 = hand
% COM6 = arm
%
% 上位机设置：
%   输出内容：角速度 + 加速度
%   通讯速率：115200
%   回传速率：200 Hz
%   带宽：188 Hz 或 98 Hz
%
% 依赖：
%   wit_parse_packet.m
%   wit_parse_stream.m
%   wit_read_gyro_acc.m
%   gyro_integrate.m
%   mahony_update_adaptive.m
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

%% 2. 采样参数

Fs = 200;
Ts = 1 / Fs;

% 模块静止 GYRO 已经输出 0，所以不做外部零偏扣除
biasHand_deg_s = [0 0 0];
biasArm_deg_s  = [0 0 0];

%% 3. Mahony 参数

mh = struct();

% % 第一版建议 Ki = 0，避免积分项引入额外慢漂
% mh.Kp = 0.5;
% mh.Ki = 0.0;
% mh.Kp = 1.5;%0.08
mh.Kp = 0.08;
mh.Ki = 0;

mh.accTol = 0.4;        % 原来 1.0，改严格
mh.gyroTolDeg = 30;     % 原来 120，改严格
mh.intLimit = 0.2;

mh.g = 9.81;

% 加速度模长距离 g 超过该阈值时，逐渐降低加速度计权重
mh.accTol = 1.0;        % m/s^2

% 角速度超过该阈值时，逐渐降低加速度计权重
mh.gyroTolDeg = 120;    % deg/s

% 积分项限幅
mh.intLimit = 0.5;

%% 4. 姿态初始化

qHand = [1 0 0 0];
qArm  = [1 0 0 0];

intErrHand = [0 0 0];
intErrArm  = [0 0 0];

latestAccHand = [];
latestAccArm  = [];

qRel0 = [1 0 0 0];

%% 5. 绘图初始化

plotParams = struct();
plotParams.targetFPS = 25;
plotParams.showWindow = 8;
plotParams.maxPoints = 500;

plotParams.thetaYLim = [0 180];
plotParams.thetaYTick = 0:30:180;

plotParams.eulerYLim = [-180 180];
plotParams.eulerYTick = -180:60:180;

P = imu_realtime_plot_init(plotParams);

disp("自适应 MH 主脚本启动。");
disp("COM5 = hand，COM6 = arm。");
disp("按 z 可重新置零。关闭图窗停止。");

tStart = tic;

%% 6. 主循环

while ishandle(P.fig)

    %% 6.1 读取 hand / arm 的 gyro + acc

    [gyroHandList, accHandList, bufferHand] = wit_read_gyro_acc(sHand, bufferHand);
    [gyroArmList,  accArmList,  bufferArm]  = wit_read_gyro_acc(sArm,  bufferArm);

    if ~isempty(accHandList)
        latestAccHand = accHandList(end,:);
    end

    if ~isempty(accArmList)
        latestAccArm = accArmList(end,:);
    end

    %% 6.2 更新 hand 姿态

    for i = 1:size(gyroHandList, 1)

        omega_deg_s = gyroHandList(i,:) - biasHand_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;

        if isempty(latestAccHand)
            % 没有加速度数据时退化为 GI
            qHand = gyro_integrate(qHand, omega_rad_s, Ts);
        else
            % 如果本轮 accList 有对应样本，优先用对应样本
            if ~isempty(accHandList)
                idx = min(i, size(accHandList,1));
                accUse = accHandList(idx,:);
            else
                accUse = latestAccHand;
            end

            [qHand, intErrHand] = mahony_update_adaptive( ...
                qHand, intErrHand, omega_rad_s, accUse, Ts, mh);
        end
    end

    %% 6.3 更新 arm 姿态

    for i = 1:size(gyroArmList, 1)

        omega_deg_s = gyroArmList(i,:) - biasArm_deg_s;
        omega_rad_s = omega_deg_s * pi / 180;

        if isempty(latestAccArm)
            qArm = gyro_integrate(qArm, omega_rad_s, Ts);
        else
            if ~isempty(accArmList)
                idx = min(i, size(accArmList,1));
                accUse = accArmList(idx,:);
            else
                accUse = latestAccArm;
            end

            [qArm, intErrArm] = mahony_update_adaptive( ...
                qArm, intErrArm, omega_rad_s, accUse, Ts, mh);
        end
    end

    %% 6.4 按 z 重新置零

    if P.fig.UserData.zeroRequested
        qHand = [1 0 0 0];
        qArm  = [1 0 0 0];

        intErrHand = [0 0 0];
        intErrArm  = [0 0 0];

        qRel0 = [1 0 0 0];

        P = imu_realtime_plot_reset(P);
        tStart = tic;

        disp("MH 已重新置零：qHand、qArm、intErr、qRel0 均重置。");
    end

    %% 6.5 计算相对姿态

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = quat_normalize(qRel);

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = quat_normalize(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    %% 6.6 更新显示

    tNow = toc(tStart);

    qHandShow = qHand;
    qArmShow  = qArm;

    P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow);

    pause(0.001);
end

%% 7. 释放串口

clear sHand sArm;
disp("程序结束，串口已释放。");