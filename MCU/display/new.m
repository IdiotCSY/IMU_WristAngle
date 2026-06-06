%% mcu_realtime_display.m
% STM32 双 IMU 姿态实时接收与绘图
%
% STM32 输出格式：
%   ATT,roll,pitch,yaw,theta,handCnt,armCnt,handBad,armBad,...
%       handOverflow,armOverflow,handHz,armHz,handDtErrUs,armDtErrUs
%
% MATLAB 功能：
%   1. 接收 STM32 输出的相对姿态；
%   2. 实时绘制欧拉角、轴角；
%   3. 实时绘制 hand / arm 的平均 dt 误差；
%   4. 按 z 向 STM32 发送置零命令；
%   5. 关闭图窗后自动退出并释放串口。

clear; clc; close all;

%% 1. 串口设置

comName = "COM9";
baudRate = 115200;

s = serialport(comName, baudRate, "Timeout", 0.05);
configureTerminator(s, "CR/LF");

% 正点原子板载 CH340 需要关闭 DTR / RTS
% 否则 MATLAB 打开串口后可能影响复位/运行状态
setDTR(s, false);
setRTS(s, false);

pause(0.5);
flush(s);

cleanupObj = onCleanup(@() cleanup_serial(s));

fprintf("已打开串口 %s，波特率 %d。\n", comName, baudRate);
fprintf("DTR = 0, RTS = 0。\n");
fprintf("在图窗内按 z：发送置零命令。\n");
fprintf("关闭图窗：结束程序并释放串口。\n\n");

%% 2. 数据缓存

maxPoints = 1200;

tBuf     = nan(maxPoints, 1);
rollBuf  = nan(maxPoints, 1);
pitchBuf = nan(maxPoints, 1);
yawBuf   = nan(maxPoints, 1);
thetaBuf = nan(maxPoints, 1);

handCntBuf = nan(maxPoints, 1);
armCntBuf  = nan(maxPoints, 1);
handBadBuf = nan(maxPoints, 1);
armBadBuf  = nan(maxPoints, 1);

handOverflowBuf = nan(maxPoints, 1);
armOverflowBuf  = nan(maxPoints, 1);

handHzBuf = nan(maxPoints, 1);
armHzBuf  = nan(maxPoints, 1);

handDtErrUsBuf = nan(maxPoints, 1);
armDtErrUsBuf  = nan(maxPoints, 1);

handDmaGyroBuf = nan(maxPoints, 1);
armDmaGyroBuf  = nan(maxPoints, 1);

handDmaAvgGyroBuf = nan(maxPoints, 1);
armDmaAvgGyroBuf  = nan(maxPoints, 1);

handDmaMaxGyroBuf = nan(maxPoints, 1);
armDmaMaxGyroBuf  = nan(maxPoints, 1);

idx = 0;
tStart = tic;

%% 3. 建立图窗

fig = figure( ...
    "Name", "STM32 Dual IMU Realtime Display", ...
    "Color", "w");

fig.UserData.stopRequested = false;
fig.UserData.serialObj = s;

fig.WindowKeyPressFcn = @(src, event) key_callback(src, event);
fig.CloseRequestFcn = @(src, event) close_callback(src, event);

tl = tiledlayout(fig, 4, 1);
tl.TileSpacing = "compact";
tl.Padding = "compact";

% 欧拉角图
axEuler = nexttile(tl, 1);
hold(axEuler, "on");
grid(axEuler, "on");

hRoll  = plot(axEuler, nan, nan, "r", "LineWidth", 1.2);
hPitch = plot(axEuler, nan, nan, "g", "LineWidth", 1.2);
hYaw   = plot(axEuler, nan, nan, "b", "LineWidth", 1.2);

xlabel(axEuler, "时间 (s)");
ylabel(axEuler, "欧拉角 (deg)");
title(axEuler, "STM32 输出相对欧拉角");
legend(axEuler, "Roll", "Pitch", "Yaw", "Location", "best");
ylim(axEuler, [-180 180]);
yticks(axEuler, -180:60:180);

% 轴角图
axTheta = nexttile(tl, 2);
hold(axTheta, "on");
grid(axTheta, "on");

hTheta = plot(axTheta, nan, nan, "k", "LineWidth", 1.3);

xlabel(axTheta, "时间 (s)");
ylabel(axTheta, "轴角 \theta (deg)");
title(axTheta, "STM32 输出相对轴角");
ylim(axTheta, [0 180]);
yticks(axTheta, 0:30:180);

% dt 误差图
axDt = nexttile(tl, 3);
hold(axDt, "on");
grid(axDt, "on");

hHandDt = plot(axDt, nan, nan, "r", "LineWidth", 1.2);
hArmDt  = plot(axDt, nan, nan, "b", "LineWidth", 1.2);

xlabel(axDt, "时间 (s)");
ylabel(axDt, "平均 dt 误差 (\mus)");
title(axDt, "实际平均 dt 与理论 dt 的误差");
legend(axDt, "hand dt error", "arm dt error", "Location", "best");

% 先给一个较宽范围，后面可以根据实际误差调
ylim(axDt, [-1000 1000]);
yticks(axDt, -1000:250:1000);

% DMA 单次接收 GYRO 帧数图
axDma = nexttile(tl, 4);
hold(axDma, "on");
grid(axDma, "on");

hHandDmaGyro = plot(axDma, nan, nan, "r", "LineWidth", 1.2);
hArmDmaGyro  = plot(axDma, nan, nan, "b", "LineWidth", 1.2);

xlabel(axDma, "时间 (s)");
ylabel(axDma, "每次 DMA 解析出的 GYRO 帧数");
title(axDma, "DMA 单次接收包含的 GYRO 帧数");
legend(axDma, "hand", "arm", "Location", "best");

ylim(axDma, [0 8]);
yticks(axDma, 0:1:8);

%% 4. 主循环：接收 + 解析 + 绘图

while true

    if ~ishandle(fig)
        break;
    end

    if isfield(fig.UserData, "stopRequested") && fig.UserData.stopRequested
        break;
    end

    if s.NumBytesAvailable <= 0
        drawnow limitrate;
        pause(0.005);
        continue;
    end

    try
        line = readline(s);
    catch
        drawnow limitrate;
        continue;
    end

    line = strtrim(line);

    % 启动提示、ZERO SET 等非 ATT 行，直接显示
    if ~startsWith(line, "ATT")
        if strlength(line) > 0
            disp(line);
        end
        drawnow limitrate;
        continue;
    end

    data = parse_att_line(line);

    if isempty(data)
        fprintf("解析失败：%s\n", line);
        drawnow limitrate;
        continue;
    end

    % data =
    % [roll pitch yaw theta handCnt armCnt handBad armBad ...
    %  handOverflow armOverflow handHz armHz handDtErrUs armDtErrUs]

    roll  = data(1);
    pitch = data(2);
    yaw   = data(3);
    theta = data(4);

    handCnt = data(5);
    armCnt  = data(6);
    handBad = data(7);
    armBad  = data(8);

    handOverflow = data(9);
    armOverflow  = data(10);

    handHz = data(11);
    armHz  = data(12);

    handDtErrUs = data(13);
    armDtErrUs  = data(14);

    handDmaBytes = data(15);
    armDmaBytes  = data(16);
    
    handDmaGyro = data(17);
    armDmaGyro  = data(18);
    
    handDmaAvgGyro = data(19);
    armDmaAvgGyro  = data(20);
    
    handDmaMaxGyro = data(21);
    armDmaMaxGyro  = data(22);

    idx = idx + 1;
    writeIdx = mod(idx - 1, maxPoints) + 1;

    tNow = toc(tStart);

    tBuf(writeIdx)     = tNow;
    rollBuf(writeIdx)  = roll;
    pitchBuf(writeIdx) = pitch;
    yawBuf(writeIdx)   = yaw;
    thetaBuf(writeIdx) = theta;

    handCntBuf(writeIdx) = handCnt;
    armCntBuf(writeIdx)  = armCnt;
    handBadBuf(writeIdx) = handBad;
    armBadBuf(writeIdx)  = armBad;

    handOverflowBuf(writeIdx) = handOverflow;
    armOverflowBuf(writeIdx)  = armOverflow;

    handHzBuf(writeIdx) = handHz;
    armHzBuf(writeIdx)  = armHz;

    handDtErrUsBuf(writeIdx) = handDtErrUs;
    armDtErrUsBuf(writeIdx)  = armDtErrUs;

    handDmaGyroBuf(writeIdx) = handDmaGyro;
    armDmaGyroBuf(writeIdx)  = armDmaGyro;
    
    handDmaAvgGyroBuf(writeIdx) = handDmaAvgGyro;
    armDmaAvgGyroBuf(writeIdx)  = armDmaAvgGyro;
    
    handDmaMaxGyroBuf(writeIdx) = handDmaMaxGyro;
    armDmaMaxGyroBuf(writeIdx)  = armDmaMaxGyro;

    % 取有效数据并按时间排序
    valid = ~isnan(tBuf);

    tPlot     = tBuf(valid);
    rollPlot  = rollBuf(valid);
    pitchPlot = pitchBuf(valid);
    yawPlot   = yawBuf(valid);
    thetaPlot = thetaBuf(valid);

    handDtPlot = handDtErrUsBuf(valid);
    armDtPlot  = armDtErrUsBuf(valid);

    handDmaGyroPlot = handDmaGyroBuf(valid);
    armDmaGyroPlot  = armDmaGyroBuf(valid);
    
    handDmaAvgGyroPlot = handDmaAvgGyroBuf(valid);
    armDmaAvgGyroPlot  = armDmaAvgGyroBuf(valid);
    
    handDmaMaxGyroPlot = handDmaMaxGyroBuf(valid);
    armDmaMaxGyroPlot  = armDmaMaxGyroBuf(valid);

    [tPlot, order] = sort(tPlot);

    rollPlot  = rollPlot(order);
    pitchPlot = pitchPlot(order);
    yawPlot   = yawPlot(order);
    thetaPlot = thetaPlot(order);

    handDtPlot = handDtPlot(order);
    armDtPlot  = armDtPlot(order);

    handDmaGyroPlot = handDmaGyroPlot(order);
    armDmaGyroPlot  = armDmaGyroPlot(order);
    
    handDmaAvgGyroPlot = handDmaAvgGyroPlot(order);
    armDmaAvgGyroPlot  = armDmaAvgGyroPlot(order);
    
    handDmaMaxGyroPlot = handDmaMaxGyroPlot(order);
    armDmaMaxGyroPlot  = armDmaMaxGyroPlot(order);

    % 更新曲线
    set(hRoll,  "XData", tPlot, "YData", rollPlot);
    set(hPitch, "XData", tPlot, "YData", pitchPlot);
    set(hYaw,   "XData", tPlot, "YData", yawPlot);
    set(hTheta, "XData", tPlot, "YData", thetaPlot);

    set(hHandDt, "XData", tPlot, "YData", handDtPlot);
    set(hArmDt,  "XData", tPlot, "YData", armDtPlot);

    set(hHandDmaGyro, "XData", tPlot, "YData", handDmaGyroPlot);
    set(hArmDmaGyro,  "XData", tPlot, "YData", armDmaGyroPlot);

    % 只显示最近 12 秒
    showWindow = 12;

    if tNow > showWindow
        xlim(axEuler, [tNow - showWindow, tNow]);
        xlim(axTheta, [tNow - showWindow, tNow]);
        xlim(axDt,    [tNow - showWindow, tNow]);
        xlim(axDma, [tNow - showWindow, tNow]);
    else
        xlim(axEuler, [0, showWindow]);
        xlim(axTheta, [0, showWindow]);
        xlim(axDt,    [0, showWindow]);
        xlim(axDma, [0, showWindow]);
    end

    title(axTheta, sprintf( ...
        "轴角 | cnt=[%d %d] bad=[%d %d] overflow=[%d %d]", ...
        handCnt, armCnt, handBad, armBad, handOverflow, armOverflow));

    title(axDt, sprintf( ...
        "dt误差 | handHz=%.2f armHz=%.2f | dtErr=[%.1f %.1f] us", ...
        handHz, armHz, handDtErrUs, armDtErrUs));

    title(axDma, sprintf( ...
    "DMA解析GYRO帧数 | last=[%d %d] avg=[%.2f %.2f] max=[%d %d]", ...
    handDmaGyro, armDmaGyro, ...
    handDmaAvgGyro, armDmaAvgGyro, ...
    handDmaMaxGyro, armDmaMaxGyro));

    drawnow limitrate;
end

if exist("fig", "var") && ishandle(fig)
    delete(fig);
end

fprintf("程序已停止。\n");

%% ===================== 局部函数 =====================

function data = parse_att_line(line)
% 解析 STM32 输出的 ATT 行
%
% 兼容三种格式：
%
% 旧格式 1：
%   ATT + 8 个数字
%
% 旧格式 2：
%   ATT + 10 个数字，包含 overflow
%
% 当前格式：
%   ATT + 22 个数字，包含 dt误差 和 DMA单次接收监测
%
% 最终统一输出 22 个数字：
%
% data =
% [roll pitch yaw theta ...
%  handCnt armCnt handBad armBad ...
%  handOverflow armOverflow ...
%  handHz armHz handDtErrUs armDtErrUs ...
%  handDmaBytes armDmaBytes ...
%  handDmaGyro armDmaGyro ...
%  handDmaAvgGyro armDmaAvgGyro ...
%  handDmaMaxGyro armDmaMaxGyro]

    data = [];

    parts = split(line, ",");

    if numel(parts) < 9
        return;
    end

    nNum = numel(parts) - 1;
    nums = zeros(1, nNum);

    for i = 1:nNum
        nums(i) = str2double(parts{i+1});
    end

    if any(isnan(nums))
        return;
    end

    % 不足 22 个数字就补 0，保证主程序索引不会报错
    if numel(nums) < 22
        nums = [nums, zeros(1, 22 - numel(nums))];
    end

    % 超过 22 个就只取前 22 个
    if numel(nums) > 22
        nums = nums(1:22);
    end

    data = nums;
end

function key_callback(src, event)
% 图窗按键回调
%
% 按 z：
%   向 STM32 发送 z + 换行。
%   STM32 收到后执行 DualIMU_SetZero()。

    if ~isfield(src.UserData, "serialObj")
        return;
    end

    s = src.UserData.serialObj;

    if strcmpi(event.Key, "z")
        try
            writeline(s, "z");
            fprintf("已发送置零命令 z。\n");
        catch ME
            fprintf("发送 z 失败：%s\n", ME.message);
        end
    end
end

function close_callback(src, ~)
% 图窗关闭回调

    if ishandle(src)
        src.UserData.stopRequested = true;
        delete(src);
    end
end

function cleanup_serial(s)
% 脚本退出时释放串口

    try
        flush(s);
    catch
    end

    try
        clear s;
        fprintf("串口已释放。\n");
    catch
    end
end