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
% 按键：
%   z : 将当前相对姿态定义为零位
%
% 注意：
%   与 GI 不同，MH 按 z 时不要强制 qHand/qArm = [1 0 0 0]。
%   因为 MH 有加速度计闭环修正，强制归零后会重新收敛，
%   可能导致静止状态下相对角自动冒出一个小误差。

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

% 你前面实测静止 GYRO 输出全 0，所以不做外部零偏扣除
biasHand_deg_s = [0 0 0];
biasArm_deg_s  = [0 0 0];

%% 3. 自适应 Mahony 参数

mh = struct();

% 当前较稳的保守参数
mh.Kp = 0.08;
mh.Ki = 0.0;

mh.g = 9.81;

% 加速度模长偏离 g 超过该范围时，降低/关闭加速度修正
mh.accTol = 0.4;        % m/s^2

% 角速度超过该阈值时，降低/关闭加速度修正
mh.gyroTolDeg = 30;     % deg/s

% 积分项限幅。当前 Ki=0，基本不起作用，但保留字段
mh.intLimit = 0.2;

%% 测试参数（记得注释）
mh.Kp = 1.5;
mh.Ki = 0.0;
mh.accTol = 1.0;        % m/s^2
mh.gyroTolDeg = 120;    % deg/s
mh.intLimit = 0.5;

%% 4. 姿态状态初始化

qHand = [1 0 0 0];
qArm  = [1 0 0 0];

intErrHand = [0 0 0];
intErrArm  = [0 0 0];

latestAccHand = [];
latestAccArm  = [];

% 相对姿态零位
qRel0 = [1 0 0 0];

% 3D 显示零位
qHand0 = [];
qArm0  = [];

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
disp("按 z：将当前相对姿态定义为零位。关闭图窗停止。");

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
            qHand = gyro_integrate(qHand, omega_rad_s, Ts);
        else
            if ~isempty(accHandList)
                idx = min(i, size(accHandList, 1));
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
                idx = min(i, size(accArmList, 1));
                accUse = accArmList(idx,:);
            else
                accUse = latestAccArm;
            end

            [qArm, intErrArm] = mahony_update_adaptive( ...
                qArm, intErrArm, omega_rad_s, accUse, Ts, mh);
        end
    end

    %% 6.4 计算当前相对姿态

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = quat_normalize(qRel);

    %% 6.5 按 z：记录当前姿态为零位

    if P.fig.UserData.zeroRequested

        % 注意：这里不重置 qHand/qArm。
        % 只把当前 hand、arm 和相对姿态记录为显示/计算零位。
        qHand0 = qHand;
        qArm0  = qArm;
        qRel0  = qRel;

        % 积分误差项可以清零，避免历史修正项继续影响后续
        intErrHand = [0 0 0];
        intErrArm  = [0 0 0];

        % 清空绘图曲线并重新计时
        P = imu_realtime_plot_reset(P);
        P.fig.UserData.zeroRequested = false;

        tStart = tic;

        disp("MH 已重新定义当前姿态为零位：未重置 qHand/qArm，只更新 qHand0/qArm0/qRel0。");
    end

    %% 6.6 计算零位补偿后的相对姿态

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = quat_normalize(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    %% 6.7 计算 3D 显示姿态

    % 如果还没按 z，就显示算法当前绝对积分姿态；
    % 按 z 后，显示相对于按 z 时刻的姿态变化。
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

    %% 6.8 更新显示

    tNow = toc(tStart);

    P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow);

    pause(0.001);
end

%% 7. 释放串口

clear sHand sArm;
disp("程序结束，串口已释放。");