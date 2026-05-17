%% analyze_limit_point_validation.m
% 限位点动态回位验证实验后处理
%
% 输入：
%   results/limit_point_validation_20260517_183323.mat
%
% 读取变量：
%   RefPoints : 6 个限位点静态参考
%   Events    : 每次动态回位验证事件
%   DataLog   : 全程连续姿态数据
%
% 输出图：
%   1. 连续相对欧拉角 + 限位点事件标记
%   2. 连续轴角 theta + 限位点事件标记
%   3. 各限位点 theta 误差随时间变化
%   4. 各限位点欧拉角误差随时间变化
%   5. 各限位点平均 theta 误差统计柱状图

clear; clc; close all;

%% 1. 加载数据

fileName = fullfile('results', 'limit_point_validation_20260517_183323.mat');

if ~isfile(fileName)
    fileName = 'limit_point_validation_20260517_183323.mat';
end

load(fileName);

fprintf("已加载文件：%s\n", fileName);

if ~exist('RefPoints', 'var') || ~exist('Events', 'var') || ~exist('DataLog', 'var')
    error("mat 文件中缺少 RefPoints / Events / DataLog，请检查保存内容。");
end

numPoints = numel(RefPoints);

if isempty(Events)
    error("Events 为空。说明还没有记录动态回位验证事件。");
end

%% 2. 提取 Events 数据

nEvent = numel(Events);

eventTime = zeros(nEvent,1);
eventPointID = zeros(nEvent,1);
thetaErr = zeros(nEvent,1);
eulErr = zeros(nEvent,3);

for k = 1:nEvent
    eventTime(k) = Events(k).time;
    eventPointID(k) = Events(k).pointID;
    thetaErr(k) = Events(k).thetaErr;
    eulErr(k,:) = Events(k).eulErr;
end

%% 3. 图1：连续相对欧拉角 + 限位点事件标记

figure('Name','连续相对欧拉角与限位点标记','Color','w');

plot(DataLog.t, DataLog.relEuler(:,1), 'r', 'LineWidth', 1.1); hold on;
plot(DataLog.t, DataLog.relEuler(:,2), 'g', 'LineWidth', 1.1);
plot(DataLog.t, DataLog.relEuler(:,3), 'b', 'LineWidth', 1.1);

grid on;
xlabel('时间 (s)');
ylabel('相对欧拉角 (deg)');
legend('Roll 相对角','Pitch 相对角','Yaw 相对角', 'Location','best');
title('连续相对欧拉角与限位点记录时刻');
ylim([-180 180]);

for k = 1:nEvent
    xline(eventTime(k), '--', sprintf('位置%d', eventPointID(k)), ...
        'LabelOrientation','horizontal');
end

%% 4. 图2：连续轴角 theta + 限位点事件标记

figure('Name','连续轴角与限位点标记','Color','w');

plot(DataLog.t, DataLog.thetaDeg, 'k', 'LineWidth', 1.2);
grid on;
xlabel('时间 (s)');
ylabel('轴角 \theta (deg)');
title('连续相对轴角 \theta 与限位点记录时刻');

thetaMax = max(DataLog.thetaDeg);
ylim([0, max(5, ceil(thetaMax/5)*5)]);

for k = 1:nEvent
    xline(eventTime(k), '--', sprintf('位置%d', eventPointID(k)), ...
        'LabelOrientation','horizontal');
end

%% 5. 图3：每个限位点 theta 误差随时间变化

figure('Name','各限位点轴角误差随时间变化','Color','w');
hold on; grid on;

for pid = 1:numPoints
    idx = eventPointID == pid;

    if any(idx)
        plot(eventTime(idx), thetaErr(idx), '-o', ...
            'LineWidth', 1.3, ...
            'DisplayName', sprintf('位置%d', pid));
    end
end

xlabel('时间 (s)');
ylabel('轴角误差 \theta_{err} (deg)');
title('各限位点轴角误差随时间变化');
legend('Location','best');
ylim([0, max(3, ceil(max(thetaErr)/1)*1)]);

%% 6. 图4：每个限位点欧拉角误差随时间变化

figure('Name','各限位点欧拉角误差随时间变化','Color','w');

for pid = 1:numPoints
    idx = eventPointID == pid;

    if ~any(idx)
        continue;
    end

    subplot(numPoints, 1, pid);

    plot(eventTime(idx), eulErr(idx,1), 'r-o', 'LineWidth', 1.1); hold on;
    plot(eventTime(idx), eulErr(idx,2), 'g-o', 'LineWidth', 1.1);
    plot(eventTime(idx), eulErr(idx,3), 'b-o', 'LineWidth', 1.1);

    grid on;
    ylabel(sprintf('位置%d误差', pid));

    if pid == 1
        title('各限位点欧拉角误差随时间变化');
        legend('Roll 误差','Pitch 误差','Yaw 误差', 'Location','best');
    end

    if pid == numPoints
        xlabel('时间 (s)');
    end
end

%% 7. 统计每个限位点误差

pointIDList = (1:numPoints).';

meanThetaErr = nan(numPoints,1);
maxThetaErr = nan(numPoints,1);
stdThetaErr = nan(numPoints,1);
numSamples = zeros(numPoints,1);

meanAbsRollErr = nan(numPoints,1);
meanAbsPitchErr = nan(numPoints,1);
meanAbsYawErr = nan(numPoints,1);

for pid = 1:numPoints
    idx = eventPointID == pid;

    if ~any(idx)
        continue;
    end

    errTheta_i = thetaErr(idx);
    errEul_i = eulErr(idx,:);

    numSamples(pid) = numel(errTheta_i);
    meanThetaErr(pid) = mean(errTheta_i);
    maxThetaErr(pid) = max(errTheta_i);
    stdThetaErr(pid) = std(errTheta_i);

    meanAbsRollErr(pid) = mean(abs(errEul_i(:,1)));
    meanAbsPitchErr(pid) = mean(abs(errEul_i(:,2)));
    meanAbsYawErr(pid) = mean(abs(errEul_i(:,3)));
end

ResultTable = table( ...
    pointIDList, ...
    numSamples, ...
    meanThetaErr, ...
    maxThetaErr, ...
    stdThetaErr, ...
    meanAbsRollErr, ...
    meanAbsPitchErr, ...
    meanAbsYawErr, ...
    'VariableNames', { ...
        '位置编号', ...
        '验证次数', ...
        '平均轴角误差_deg', ...
        '最大轴角误差_deg', ...
        '轴角误差标准差_deg', ...
        '平均Roll绝对误差_deg', ...
        '平均Pitch绝对误差_deg', ...
        '平均Yaw绝对误差_deg' ...
    } ...
);

disp("===== 限位点动态回位验证结果 =====");
disp(ResultTable);

fprintf("\n===== 总体结果 =====\n");
fprintf("总验证次数: %d\n", nEvent);
fprintf("平均轴角误差 = %.4f deg\n", mean(thetaErr));
fprintf("最大轴角误差 = %.4f deg\n", max(thetaErr));
fprintf("轴角误差标准差 = %.4f deg\n", std(thetaErr));

%% 8. 图5：各限位点 theta 误差统计柱状图

figure('Name','各限位点轴角误差统计','Color','w');

bar(pointIDList, meanThetaErr);
hold on;
errorbar(pointIDList, meanThetaErr, stdThetaErr, ...
    'k', 'LineStyle','none', 'LineWidth',1.2);

grid on;
xlabel('限位点编号');
ylabel('轴角误差 (deg)');
title('各限位点平均轴角误差');
xticks(pointIDList);
xticklabels(arrayfun(@(x) sprintf('位置%d', x), pointIDList, 'UniformOutput', false));

%% 9. 保存统计结果

outDir = 'results';

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

outTableName = fullfile(outDir, 'limit_point_validation_summary.csv');
writetable(ResultTable, outTableName);

fprintf("\n统计表已保存：%s\n", outTableName);