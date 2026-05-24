%% mcu_realtime_display.m
% STM32 双 IMU 姿态实时接收与绘图
%
% STM32 输出格式：
%   ATT,roll,pitch,yaw,theta,handCnt,armCnt,handBad,armBad
%
% MATLAB 功能：
%   1. 接收 STM32 发来的 ATT 数据；
%   2. 实时绘制 roll / pitch / yaw / theta；
%   3. 按 z 向 STM32 发送置零命令；
%   4. 关闭图窗后自动退出程序并释放串口。

clear; clc; close all;

%% 1. 串口设置

comName = "COM9";
baudRate = 115200;

s = serialport(comName, baudRate, "Timeout", 0.05);

% STM32 输出 \r\n，这里用 CR/LF 更严格
configureTerminator(s, "CR/LF");

% 正点原子板载 CH340：必须关闭 DTR / RTS
% 否则可能影响复位/运行状态，导致 MATLAB 收不到数据
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

maxPoints = 1000;

tBuf     = nan(maxPoints, 1);
rollBuf  = nan(maxPoints, 1);
pitchBuf = nan(maxPoints, 1);
yawBuf   = nan(maxPoints, 1);
thetaBuf = nan(maxPoints, 1);

handCntBuf = nan(maxPoints, 1);
armCntBuf  = nan(maxPoints, 1);
handBadBuf = nan(maxPoints, 1);
armBadBuf  = nan(maxPoints, 1);

idx = 0;
tStart = tic;

%% 3. 建立图窗

fig = figure( ...
    "Name", "STM32 Dual IMU Realtime Display", ...
    "Color", "w");

% 用 UserData 存运行状态和串口对象
fig.UserData.stopRequested = false;
fig.UserData.serialObj = s;

% WindowKeyPressFcn 比 KeyPressFcn 更稳定
fig.WindowKeyPressFcn = @(src, event) key_callback(src, event);

% 关闭图窗时先设置停止标志，再删除图窗
fig.CloseRequestFcn = @(src, event) close_callback(src, event);

tl = tiledlayout(fig, 2, 1);
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

%% 4. 主循环：接收 + 解析 + 绘图

while true

    % 图窗已关闭或请求停止，则退出循环
    if ~ishandle(fig)
        break;
    end

    if isfield(fig.UserData, "stopRequested") && fig.UserData.stopRequested
        break;
    end

    % 没有数据时也要 drawnow，让按键/关闭事件能被处理
    if s.NumBytesAvailable <= 0
        drawnow limitrate;
        pause(0.005);
        continue;
    end

    % 读取一行。若偶发超时，跳过本轮
    try
        line = readline(s);
    catch
        drawnow limitrate;
        continue;
    end

    line = strtrim(line);

    % 非 ATT 行，例如启动提示、ZERO SET，直接显示
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

    roll  = data(1);
    pitch = data(2);
    yaw   = data(3);
    theta = data(4);

    handCnt = data(5);
    armCnt  = data(6);
    handBad = data(7);
    armBad  = data(8);

    % 写入环形缓存
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

    % 按时间顺序取有效数据
    valid = ~isnan(tBuf);

    tPlot     = tBuf(valid);
    rollPlot  = rollBuf(valid);
    pitchPlot = pitchBuf(valid);
    yawPlot   = yawBuf(valid);
    thetaPlot = thetaBuf(valid);

    [tPlot, order] = sort(tPlot);

    rollPlot  = rollPlot(order);
    pitchPlot = pitchPlot(order);
    yawPlot   = yawPlot(order);
    thetaPlot = thetaPlot(order);

    % 更新曲线
    set(hRoll,  "XData", tPlot, "YData", rollPlot);
    set(hPitch, "XData", tPlot, "YData", pitchPlot);
    set(hYaw,   "XData", tPlot, "YData", yawPlot);
    set(hTheta, "XData", tPlot, "YData", thetaPlot);

    % 只显示最近 12 秒
    showWindow = 12;

    if tNow > showWindow
        xlim(axEuler, [tNow - showWindow, tNow]);
        xlim(axTheta, [tNow - showWindow, tNow]);
    else
        xlim(axEuler, [0, showWindow]);
        xlim(axTheta, [0, showWindow]);
    end

    title(axTheta, sprintf( ...
        "STM32 输出相对轴角 | handCnt=%d armCnt=%d handBad=%d armBad=%d", ...
        handCnt, armCnt, handBad, armBad));

    drawnow limitrate;
end

% 如果是通过 stopRequested 退出，但图窗还在，则删除图窗
if exist("fig", "var") && ishandle(fig)
    delete(fig);
end

fprintf("程序已停止。\n");

%% ===================== 局部函数 =====================

function data = parse_att_line(line)
% 解析 STM32 输出的 ATT 行
%
% 输入示例：
%   ATT,1.234,-0.520,3.100,3.420,1234,1235,0,0
%
% 输出：
%   data = [roll pitch yaw theta handCnt armCnt handBad armBad]

    data = [];

    parts = split(line, ",");

    if numel(parts) < 9
        return;
    end

    nums = zeros(1, 8);

    for i = 1:8
        nums(i) = str2double(parts{i+1});
    end

    if any(isnan(nums))
        return;
    end

    data = nums;
end

function key_callback(src, event)
% 图窗按键回调
%
% 按 z：
%   向 STM32 发送 ASCII 字符 z，并附带换行。
%
% 用 writeline 比只 write 一个 uint8 更稳，
% 因为 STM32 端 USART1 使用 ReceiveToIdle_DMA，
% 发送 z + 换行更容易触发一段完整接收事件。

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
%
% 不直接让程序卡死在 while 里。
% 先设置 stopRequested，主循环检测到后退出。

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