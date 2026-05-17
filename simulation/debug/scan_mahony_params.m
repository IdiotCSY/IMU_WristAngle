%% scan_mahony_params.m
% Mahony 参数扫描：Kp / Ki
% 前提：
% 1. Mahony 模块的 Kp、Ki 由 imu.mahonyKp / imu.mahonyKi 输入
% 2. Variant 选择 Mahony，即 ESTIMATOR = 2

clear; clc;

paths = setup_project_paths();

init_wrist_imu_sim;

model = 'wrist_imu_main';
modelPath = fullfile(paths.simulink, [model '.slx']);

bdclose all;
load_system(modelPath);

% 强制选择 Mahony
ESTIMATOR = 2;
assignin('base','ESTIMATOR',ESTIMATOR);

% 参数扫描范围
Kp_list = [0.05 0.1 0.15 0.2 0.5 1 2 4 8];
Ki_list = [0 0.005 0.01 0.02 0.05 0.1 0.15 0.20 0.3 0.4];

results = [];

idx = 1;

for i = 1:length(Kp_list)
    for j = 1:length(Ki_list)

        Kp = Kp_list(i);
        Ki = Ki_list(j);

        imu.mahonyKp = Kp;
        imu.mahonyKi = Ki;
        assignin('base','imu',imu);

        out = sim(model);

        eul_err = get_ts_data(out.eul_error_deg_out, 3);
        att_err = get_ts_data(out.attitude_error_deg_out, 1);

        rmse_roll  = sqrt(mean(eul_err(:,1).^2));
        rmse_pitch = sqrt(mean(eul_err(:,2).^2));
        rmse_yaw   = sqrt(mean(eul_err(:,3).^2));
        rmse_att   = sqrt(mean(att_err.^2));

        results(idx).Kp = Kp;
        results(idx).Ki = Ki;
        results(idx).rmse_roll = rmse_roll;
        results(idx).rmse_pitch = rmse_pitch;
        results(idx).rmse_yaw = rmse_yaw;
        results(idx).rmse_att = rmse_att;

        fprintf('Kp=%.3f, Ki=%.3f | Roll=%.4f, Pitch=%.4f, Yaw=%.4f, Att=%.4f deg\n', ...
            Kp, Ki, rmse_roll, rmse_pitch, rmse_yaw, rmse_att);

        idx = idx + 1;
    end
end

T = struct2table(results);

disp(' ');
disp('===== 按总姿态误差 RMSE 排序 =====');
T_sorted = sortrows(T, 'rmse_att');
disp(T_sorted);

best = T_sorted(1,:);

fprintf('\n最优参数：Kp = %.3f, Ki = %.3f\n', best.Kp, best.Ki);
fprintf('总姿态 RMSE = %.4f deg\n', best.rmse_att);

%% 画热力图：总姿态误差

rmse_grid = reshape([results.rmse_att], length(Ki_list), length(Kp_list))';

figure('Name','Mahony Parameter Scan');

imagesc(Ki_list, Kp_list, rmse_grid);
colorbar;
xlabel('Ki');
ylabel('Kp');
title('Total Attitude RMSE (deg)');
set(gca, 'YDir', 'normal');

%% 画不同误差排序柱状图

figure('Name','Best Mahony RMSE');

bar([best.rmse_roll, best.rmse_pitch, best.rmse_yaw, best.rmse_att]);
grid on;
set(gca, 'XTickLabel', {'Roll','Pitch','Yaw','Total'});
ylabel('RMSE (deg)');
title(sprintf('Best Mahony Params: Kp=%.3f, Ki=%.3f', best.Kp, best.Ki));

%% ===== 局部函数 =====

function data = get_ts_data(x, ncol)

    if isa(x, 'timeseries')
        data = x.Data;
    elseif isstruct(x)
        data = x.signals.values;
    else
        data = x;
    end

    data = squeeze(data);

    if ncol == 1
        data = data(:);
        return;
    end

    if size(data,2) == ncol
        return;
    elseif size(data,1) == ncol
        data = data.';
    else
        error('数据维度异常，期望 N×%d 或 %d×N，实际尺寸为：%s', ...
            ncol, ncol, mat2str(size(data)));
    end
end