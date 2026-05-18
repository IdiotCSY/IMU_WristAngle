%% MH_validate_points.m
% 双 IMU 自适应 MH + 6 个物理限位点验证实验
%
% COM5 = hand
% COM6 = arm
%
% 按键：
%   z   : 将当前相对姿态定义为零位，并清空参考点和验证事件
%   1~6 : 记录/验证对应限位点
%         第一次按：保存该点静态参考值
%         后续按：记录一次动态回位误差
%   s   : 保存实验数据到 results/
%
% 上位机设置：
%   输出内容：角速度 + 加速度
%   通讯速率：115200
%   回传速率：200 Hz
%   带宽：188 Hz 或 98 Hz

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

% 静止 GYRO 已经输出 0，所以不额外扣零偏
biasHand_deg_s = [0 0 0];
biasArm_deg_s  = [0 0 0];

%% 3. 自适应 MH 参数

mh = struct();

mh.Kp = 1.5;
mh.Ki = 0.0;
mh.g = 9.81;
mh.accTol = 1.0;        % m/s^2
mh.gyroTolDeg = 120;   % deg/s
mh.intLimit = 0.5;

%% 4. 姿态初始化

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

%% 5. 参考点与事件记录

RefPoints = init_ref_points(6);
Events = init_events();

% 连续数据记录，用于保存和后处理
DataLog.t = [];
DataLog.qOut = [];
DataLog.relEuler = [];
DataLog.thetaDeg = [];

% 最近一小段历史，用于按键时取均值，避免只取瞬时噪声
Hist.t = [];
Hist.qOut = [];
Hist.relEuler = [];
Hist.thetaDeg = [];

stableWindow = 1.0;   % 按 1~6 时，取最近 1 秒数据均值

%% 6. 绘图初始化

plotParams = struct();
plotParams.targetFPS = 25;
plotParams.showWindow = 12;
plotParams.maxPoints = 800;

% 轴角范围：0~180 deg
plotParams.thetaYLim = [0 180];
plotParams.thetaYTick = 0:30:180;

plotParams.eulerYLim = [-180 180];
plotParams.eulerYTick = -180:60:180;

P = imu_realtime_plot_init(plotParams);

% 覆盖默认按键函数：增加 1~6 和 s
P.fig.UserData.zeroRequested = false;
P.fig.UserData.saveRequested = false;
P.fig.UserData.pendingKeys = {};
set(P.fig, 'KeyPressFcn', @validate_key_callback);

disp("限位点验证实验启动。");
disp("COM5 = hand，COM6 = arm。");
disp("按 z：重新定义当前相对姿态为零位。");
disp("按 1~6：记录/验证限位点。");
disp("按 s：保存数据。");

tStart = tic;

%% 7. 主循环

while ishandle(P.fig)

    %% 7.1 读取 hand / arm 的 gyro + acc

    [gyroHandList, accHandList, bufferHand] = wit_read_gyro_acc(sHand, bufferHand);
    [gyroArmList,  accArmList,  bufferArm]  = wit_read_gyro_acc(sArm,  bufferArm);

    if ~isempty(accHandList)
        latestAccHand = accHandList(end,:);
    end

    if ~isempty(accArmList)
        latestAccArm = accArmList(end,:);
    end

    %% 7.2 更新 hand 姿态

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

    %% 7.3 更新 arm 姿态

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

    %% 7.4 当前相对姿态

    qRel = quat_mul(quat_conj(qArm), qHand);
    qRel = quat_normalize(qRel);

    %% 7.5 按 z：重新定义当前姿态为零位

    if P.fig.UserData.zeroRequested

        % MH 不再强制 qHand/qArm = [1 0 0 0]
        % 只把当前 hand、arm 以及相对姿态记录为零位
        qHand0 = qHand;
        qArm0  = qArm;
        qRel0  = qRel;

        % 清空积分误差项，避免历史修正残留
        intErrHand = [0 0 0];
        intErrArm  = [0 0 0];

        % 清空参考点和验证事件
        RefPoints = init_ref_points(6);
        Events = init_events();

        DataLog.t = [];
        DataLog.qOut = [];
        DataLog.relEuler = [];
        DataLog.thetaDeg = [];

        Hist.t = [];
        Hist.qOut = [];
        Hist.relEuler = [];
        Hist.thetaDeg = [];

        P = imu_realtime_plot_reset(P);
        P.fig.UserData.zeroRequested = false;
        P.fig.UserData.saveRequested = false;
        P.fig.UserData.pendingKeys = {};

        tStart = tic;

        disp("已重新定义当前相对姿态为零位，并清空参考点与验证事件。");
    end

    %% 7.6 计算零位补偿后的相对姿态

    qOut = quat_mul(quat_conj(qRel0), qRel);
    qOut = quat_normalize(qOut);

    relEuler = quat_to_eulZYX_deg(qOut);
    [~, thetaDeg] = quat_to_axis_angle_deg(qOut);

    tNow = toc(tStart);

    %% 7.7 计算 3D 显示姿态

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

    %% 7.8 记录连续数据

    DataLog.t(end+1,1) = tNow;
    DataLog.qOut(end+1,:) = qOut;
    DataLog.relEuler(end+1,:) = relEuler;
    DataLog.thetaDeg(end+1,1) = thetaDeg;

    Hist.t(end+1,1) = tNow;
    Hist.qOut(end+1,:) = qOut;
    Hist.relEuler(end+1,:) = relEuler;
    Hist.thetaDeg(end+1,1) = thetaDeg;

    % 历史只保留最近 5 秒，防止无限增长
    keepIdx = Hist.t >= max(0, tNow - 5);
    Hist.t = Hist.t(keepIdx);
    Hist.qOut = Hist.qOut(keepIdx,:);
    Hist.relEuler = Hist.relEuler(keepIdx,:);
    Hist.thetaDeg = Hist.thetaDeg(keepIdx);

    %% 7.9 处理 1~6 按键

    while ~isempty(P.fig.UserData.pendingKeys)
        pointID = P.fig.UserData.pendingKeys{1};
        P.fig.UserData.pendingKeys(1) = [];

        [qMean, eulMean, thetaMean] = get_recent_pose_mean(Hist, tNow, stableWindow);

        if ~RefPoints(pointID).isSet
            % 第一次按：保存为该限位点静态参考
            RefPoints(pointID).isSet = true;
            RefPoints(pointID).id = pointID;
            RefPoints(pointID).time = tNow;
            RefPoints(pointID).qRef = qMean;
            RefPoints(pointID).eulRef = eulMean;
            RefPoints(pointID).thetaRef = thetaMean;

            fprintf("位置%d参考值已保存：Euler = [%.3f %.3f %.3f] deg, theta = %.3f deg\n", ...
                pointID, eulMean(1), eulMean(2), eulMean(3), thetaMean);

            mark_event_on_plot(P, tNow, pointID, "ref", NaN);

        else
            % 后续按：记录动态回位验证
            qErr = quat_mul(quat_conj(RefPoints(pointID).qRef), qMean);
            qErr = quat_normalize(qErr);

            eulErr = quat_to_eulZYX_deg(qErr);
            [~, thetaErr] = quat_to_axis_angle_deg(qErr);

            newEvent = struct();
            newEvent.time = tNow;
            newEvent.pointID = pointID;
            newEvent.qNow = qMean;
            newEvent.eulNow = eulMean;
            newEvent.thetaNow = thetaMean;
            newEvent.qRef = RefPoints(pointID).qRef;
            newEvent.eulRef = RefPoints(pointID).eulRef;
            newEvent.thetaRef = RefPoints(pointID).thetaRef;
            newEvent.qErr = qErr;
            newEvent.eulErr = eulErr;
            newEvent.thetaErr = thetaErr;

            Events(end+1) = newEvent; %#ok<SAGROW>

            fprintf("位置%d验证已记录：轴角误差 = %.3f deg，Euler误差 = [%.3f %.3f %.3f] deg\n", ...
                pointID, thetaErr, eulErr(1), eulErr(2), eulErr(3));

            mark_event_on_plot(P, tNow, pointID, "test", thetaErr);
        end
    end

    %% 7.10 保存数据

    if P.fig.UserData.saveRequested
        P.fig.UserData.saveRequested = false;

        if ~exist('results', 'dir')
            mkdir('results');
        end

        filename = fullfile('results', ...
            ['limit_point_validation_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);

        save(filename, ...
            'RefPoints', 'Events', 'DataLog', ...
            'mh', 'Fs', 'Ts', 'comHand', 'comArm');

        fprintf("实验数据已保存：%s\n", filename);

        print_event_summary(Events);
    end

    %% 7.11 更新显示

    P = imu_realtime_plot_update(P, qHandShow, qArmShow, relEuler, thetaDeg, tNow);

    pause(0.001);
end

%% 8. 释放串口

clear sHand sArm;
disp("程序结束，串口已释放。");

%% ================= 局部函数 =================

function validate_key_callback(src, event)

key = event.Key;

if strcmp(key, 'z')
    src.UserData.zeroRequested = true;
    disp("收到 z：准备重新定义当前相对姿态为零位。");
    return;
end

if strcmp(key, 's')
    src.UserData.saveRequested = true;
    disp("收到 s：准备保存实验数据。");
    return;
end

pointID = parse_point_key(key);

if ~isempty(pointID)
    keys = src.UserData.pendingKeys;
    keys{end+1} = pointID;
    src.UserData.pendingKeys = keys;
    fprintf("收到按键 %d：准备记录/验证 位置%d。\n", pointID, pointID);
end

end

function pointID = parse_point_key(key)

pointID = [];

validKeys = {'1','2','3','4','5','6'};

if any(strcmp(key, validKeys))
    pointID = str2double(key);
    return;
end

% 兼容小键盘
if startsWith(key, 'numpad')
    numStr = erase(key, 'numpad');
    val = str2double(numStr);

    if ~isnan(val) && val >= 1 && val <= 6
        pointID = val;
    end
end

end

function RefPoints = init_ref_points(n)

template = struct();
template.isSet = false;
template.id = NaN;
template.time = NaN;
template.qRef = [NaN NaN NaN NaN];
template.eulRef = [NaN NaN NaN];
template.thetaRef = NaN;

RefPoints = repmat(template, n, 1);

end

function Events = init_events()

Events = struct( ...
    'time', {}, ...
    'pointID', {}, ...
    'qNow', {}, ...
    'eulNow', {}, ...
    'thetaNow', {}, ...
    'qRef', {}, ...
    'eulRef', {}, ...
    'thetaRef', {}, ...
    'qErr', {}, ...
    'eulErr', {}, ...
    'thetaErr', {} ...
);

end

function [qMean, eulMean, thetaMean] = get_recent_pose_mean(Hist, tNow, stableWindow)

if isempty(Hist.t)
    qMean = [1 0 0 0];
    eulMean = [0 0 0];
    thetaMean = 0;
    return;
end

idx = Hist.t >= max(0, tNow - stableWindow);

if ~any(idx)
    idx = length(Hist.t);
end

qList = Hist.qOut(idx,:);

qMean = quat_average(qList);
eulMean = quat_to_eulZYX_deg(qMean);
[~, thetaMean] = quat_to_axis_angle_deg(qMean);

end

function qMean = quat_average(qList)
% 简单四元数平均：先统一符号，再求均值归一化
% 对短时间窗口、姿态变化不大的情况够用

if isempty(qList)
    qMean = [1 0 0 0];
    return;
end

qRef = qList(1,:);

for i = 1:size(qList,1)
    if dot(qList(i,:), qRef) < 0
        qList(i,:) = -qList(i,:);
    end
end

qMean = mean(qList, 1);
qMean = quat_normalize(qMean);

end

function mark_event_on_plot(P, tNow, pointID, modeName, thetaErr)

if ~ishandle(P.fig)
    return;
end

if modeName == "ref"
    label = sprintf("位置%d参考", pointID);
else
    label = sprintf("位置%d %.2f°", pointID, thetaErr);
end

try
    xline(P.axEuler, tNow, '--', label, 'LabelOrientation','horizontal');
    xline(P.axTheta, tNow, '--', label, 'LabelOrientation','horizontal');
catch
    % 如果 MATLAB 版本不支持 xline 的某些参数，就退化为普通竖线
    yl1 = ylim(P.axEuler);
    line(P.axEuler, [tNow tNow], yl1, 'LineStyle','--');

    yl2 = ylim(P.axTheta);
    line(P.axTheta, [tNow tNow], yl2, 'LineStyle','--');
end

end

function print_event_summary(Events)

if isempty(Events)
    disp("暂无验证事件。");
    return;
end

fprintf("\n===== 限位点验证事件统计 =====\n");

pointIDs = unique([Events.pointID]);

for i = 1:numel(pointIDs)
    pid = pointIDs(i);
    idx = [Events.pointID] == pid;
    errs = [Events(idx).thetaErr];

    fprintf("位置%d: N = %d, mean = %.3f deg, max = %.3f deg, std = %.3f deg\n", ...
        pid, numel(errs), mean(errs), max(errs), std(errs));
end

allErrs = [Events.thetaErr];

fprintf("--------------------------------\n");
fprintf("全部位置: N = %d, mean = %.3f deg, max = %.3f deg, std = %.3f deg\n", ...
    numel(allErrs), mean(allErrs), max(allErrs), std(allErrs));
fprintf("================================\n\n");

end