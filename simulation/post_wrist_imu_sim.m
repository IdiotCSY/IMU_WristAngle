%% post_wrist_imu_sim.m
% 双IMU手腕相对姿态仿真后处理
% 运行前确保：
% 1. init_wrist_imu_sim 已运行
% 2. wrist_imu_main.slx 已搭建完成

paths = setup_project_paths();

%% 1. 运行 Simulink 模型

model = 'wrist_imu_main';
modelPath = fullfile(paths.simulink, [model '.slx']);

bdclose all;
load_system(modelPath);

out = sim(model);

%% 2. 读取数据

t = out.tout;

% 手腕欧拉角：估计值与真值，单位 deg
eul_est  = get_ts_data(out.eul_wrist_est_out, 3);
eul_true = get_ts_data(eul_wrist_true_ts, 3);

% 欧拉角误差，单位 deg
eul_err = get_ts_data(out.eul_error_deg_out, 3);

% 四元数姿态误差角，单位 deg
att_err = get_ts_data(out.attitude_error_deg_out, 1);

% 陀螺仪模拟输出，单位 rad/s
omega_arm_meas  = get_ts_data(out.omega_arm_meas_out, 3);
omega_hand_meas = get_ts_data(out.omega_hand_meas_out, 3);

% 加速度计模拟输出，单位 m/s^2
hasAcc = ismember('acc_arm_meas_out', out.who) && ismember('acc_hand_meas_out', out.who);

if hasAcc
    acc_arm_meas  = get_ts_data(out.acc_arm_meas_out, 3);
    acc_hand_meas = get_ts_data(out.acc_hand_meas_out, 3);
end

%% 3. 计算误差指标

rmse_roll  = sqrt(mean(eul_err(:,1).^2));
rmse_pitch = sqrt(mean(eul_err(:,2).^2));
rmse_yaw   = sqrt(mean(eul_err(:,3).^2));
rmse_att   = sqrt(mean(att_err.^2));

fprintf('\n===== 仿真误差结果 =====\n');
fprintf('Roll  RMSE = %.4f deg\n', rmse_roll);
fprintf('Pitch RMSE = %.4f deg\n', rmse_pitch);
fprintf('Yaw   RMSE = %.4f deg\n', rmse_yaw);
fprintf('Total attitude RMSE = %.4f deg\n', rmse_att);

%% 4. 绘图：真实值 vs 估计值

figure('Name','True vs Estimated Wrist Angles');

subplot(3,1,1);
plot(t, eul_true(:,1), '--', 'LineWidth', 1.2); hold on;
plot(t, eul_est(:,1), 'LineWidth', 1.2);
grid on;
ylabel('Roll (deg)');
legend('True','Estimated');

subplot(3,1,2);
plot(t, eul_true(:,2), '--', 'LineWidth', 1.2); hold on;
plot(t, eul_est(:,2), 'LineWidth', 1.2);
grid on;
ylabel('Pitch (deg)');
legend('True','Estimated');

subplot(3,1,3);
plot(t, eul_true(:,3), '--', 'LineWidth', 1.2); hold on;
plot(t, eul_est(:,3), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Yaw (deg)');
legend('True','Estimated');

sgtitle('Wrist Relative Angles: True vs Estimated');

%% 5. 绘图：欧拉角误差

figure('Name','Euler Angle Error');

plot(t, eul_err(:,1), 'LineWidth', 1.2); hold on;
plot(t, eul_err(:,2), 'LineWidth', 1.2);
plot(t, eul_err(:,3), 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Euler angle error (deg)');
legend('Roll error','Pitch error','Yaw error');
title('Euler Angle Error');

%% 6. 绘图：总姿态误差角

figure('Name','Relative Attitude Error');

plot(t, att_err, 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Attitude error (deg)');
title('Relative Attitude Error Angle');

% %% 7. 绘图：IMU模拟陀螺仪输出
% 
% deg = pi/180;
% 
% figure('Name','Simulated Gyroscope Outputs');
% 
% subplot(2,1,1);
% plot(t, omega_arm_meas/deg, 'LineWidth', 1.1);
% grid on;
% ylabel('\omega_{arm} (deg/s)');
% legend('\omega_x','\omega_y','\omega_z');
% title('Arm IMU Gyroscope Output');
% 
% subplot(2,1,2);
% plot(t, omega_hand_meas/deg, 'LineWidth', 1.1);
% grid on;
% xlabel('Time (s)');
% ylabel('\omega_{hand} (deg/s)');
% legend('\omega_x','\omega_y','\omega_z');
% title('Hand IMU Gyroscope Output');
% 
% %% 8. 绘图：IMU模拟加速度计输出
% 
% if hasAcc
%     figure('Name','Simulated Accelerometer Outputs');
% 
%     subplot(2,1,1);
%     plot(t, acc_arm_meas, 'LineWidth', 1.1);
%     grid on;
%     ylabel('a_{arm} (m/s^2)');
%     legend('a_x','a_y','a_z');
%     title('Arm IMU Accelerometer Output');
% 
%     subplot(2,1,2);
%     plot(t, acc_hand_meas, 'LineWidth', 1.1);
%     grid on;
%     xlabel('Time (s)');
%     ylabel('a_{hand} (m/s^2)');
%     legend('a_x','a_y','a_z');
%     title('Hand IMU Accelerometer Output');
% 
%     figure('Name','Accelerometer Norm');
% 
%     plot(t, vecnorm(acc_arm_meas, 2, 2), 'LineWidth', 1.2); hold on;
%     plot(t, vecnorm(acc_hand_meas, 2, 2), 'LineWidth', 1.2);
%     yline(9.81, '--', '9.81 m/s^2');
%     grid on;
%     xlabel('Time (s)');
%     ylabel('|a| (m/s^2)');
%     legend('Arm','Hand','Gravity');
%     title('Accelerometer Norm');
% else
%     warning('未检测到 acc_arm_meas_out / acc_hand_meas_out，跳过加速度计绘图。');
% end

%% ===== 局部函数 =====

function data = get_ts_data(x, ncol)
% 将 Simulink 输出统一整理成 N × ncol
% 兼容：
% 1. timeseries: Data 可能是 N×m, m×1×N, 1×m×N
% 2. structure with time
% 3. 普通数组

    if isa(x, 'timeseries')
        data = x.Data;
    elseif isstruct(x)
        data = x.signals.values;
    else
        data = x;
    end

    data = squeeze(data);

    % 标量信号
    if ncol == 1
        data = data(:);
        return;
    end

    % 常见情况：
    % N×3：直接用
    % 3×N：转置
    % 3×1×N：squeeze 后变成 3×N，再转置
    if size(data,2) == ncol
        return;
    elseif size(data,1) == ncol
        data = data.';
    else
        error('数据维度异常，期望 N×%d 或 %d×N，实际尺寸为：%s', ...
            ncol, ncol, mat2str(size(data)));
    end
end