%% init_wrist_imu.m
% 初始化双IMU手腕相对姿态仿真
% 作用：
% 1. 设置仿真参数
% 2. 调用 motion_case_01 或者其他的运动案例（需手动修改）生成真实运动
% 3. 设置IMU误差参数
% 4. 将数据打包成 timeseries，供 Simulink 使用

clear; clc; close all;

cfg.motionCase = 'motion_case_01';  %调用运动案例1
cfg.estimator = 'GI';               %调用GI算法

paths = setup_project_paths();

%% 选择算法控制器
switch upper(cfg.estimator)
    case 'GI'
        ESTIMATOR = 1;
    case 'MAHONY'
        ESTIMATOR = 2;
    case 'MADGWICK'
        ESTIMATOR = 3;
    otherwise
        error('未知姿态估计算法：%s', cfg.estimator);
end

assignin('base','ESTIMATOR',ESTIMATOR);
assignin('base','cfg',cfg);

%% 1. 基本仿真参数

cfg.Ts = 0.01;          % 采样周期 s
cfg.Fs = 1 / cfg.Ts;    % 采样频率 Hz
cfg.T  = 200;            % 仿真总时长 s

deg = pi / 180;

rng(1);                 % 固定随机种子，保证每次仿真结果一致

%% 2. 生成真实运动轨迹

trajFcn = str2func(cfg.motionCase);
traj = trajFcn(cfg);

t = traj.t;

%% 3. IMU误差参数

imu = struct();

mg  = 1e-3 * 9.81;

% ===== Gyroscope =====
% WT-IMU63 参数：
% 量程：±2000 deg/s
% 分辨率：0.061 deg/s/LSB
% RMS噪声：0.028~0.07 deg/s-rms
% 静止零漂：±0.5~1 deg/s
% 零偏稳定性：约 3.19 deg/h

% 真实静止零偏，取规格范围内的中等值
imu.gyroBias_arm  = [ 0.60, -0.45,  0.50] * deg;
imu.gyroBias_hand = [-0.55,  0.40,  0.65] * deg;

% 标定后残余零偏
% 这里假设经过静态标定后，零偏大部分被消除，但仍保留 0.02~0.04 deg/s 量级残差
imu.gyroResidualBias_arm  = [ 0.030, -0.020,  0.025] * deg;
imu.gyroResidualBias_hand = [-0.025,  0.020, -0.030] * deg;

% AE 里使用的零偏估计值
% omega_meas - gyroBiasEst = omega_true + residual_bias + noise
imu.gyroBiasEst_arm  = imu.gyroBias_arm  - imu.gyroResidualBias_arm;
imu.gyroBiasEst_hand = imu.gyroBias_hand - imu.gyroResidualBias_hand;

% 陀螺仪白噪声，数据手册 DLPF=100Hz 约 0.05 deg/s-rms
imu.gyroNoiseStd = 0.05 * deg;

% 每轴噪声标准差，后续生成噪声用这个
imu.gyroNoiseStd_arm  = [0.05, 0.05, 0.05] * deg;
imu.gyroNoiseStd_hand = [0.05, 0.05, 0.05] * deg;

% ===== Accelerometer =====
% WT-IMU63 参数：
% 量程：±16g
% 分辨率：0.0005g/LSB
% RMS噪声：0.75~1 mg-rms
% 静止零漂：±20~40 mg
% 温漂：±0.15 mg/℃

% 真实加速度计静止零漂，取 20~40 mg 量级
imu.accBias_arm  = [ 30, -25,  35] * mg;
imu.accBias_hand = [-35,  20, -40] * mg;

% 标定后残余偏置
% 假设经过标定后仍保留 3~6 mg 残差
imu.accResidualBias_arm  = [ 4, -3,  5] * mg;
imu.accResidualBias_hand = [-4,  3, -6] * mg;

% AE 里使用的加速度计偏置估计值
% acc_meas - accBiasEst = R'g + residual_bias + noise
imu.accBiasEst_arm  = imu.accBias_arm  - imu.accResidualBias_arm;
imu.accBiasEst_hand = imu.accBias_hand - imu.accResidualBias_hand;

% 加速度计白噪声，取 1 mg-rms
imu.accNoiseStd = 1 * mg;

% 每轴噪声标准差
imu.accNoiseStd_arm  = [1, 1, 1] * mg;
imu.accNoiseStd_hand = [1, 1, 1] * mg;

% 重力加速度，后续加速度计模型使用
imu.g = 9.81;
imu.g_W = [0, 0, imu.g];

%% ===== Other parameters =====
imu.g = 9.81;
imu.g_W = [0, 0, imu.g];

% 安装误差，第一版先设为0，后续再加
imu.mountError_arm_deg  = [0, 0, 0];    % roll pitch yaw
imu.mountError_hand_deg = [0, 0, 0];

%MH使用的PI控制器参数
imu.mahonyKp = 0.05;
imu.mahonyKi = 0.05;

%% 4. 打包成 timeseries，送入 Simulink

% 真实姿态，四元数格式 [w x y z]
q_arm_true_ts   = timeseries(traj.q_arm_true, t);
q_hand_true_ts  = timeseries(traj.q_hand_true, t);
q_wrist_true_ts = timeseries(traj.q_wrist_true, t);

% 真实角速度，单位 rad/s
omega_arm_true_ts  = timeseries(traj.omega_arm_true, t);
omega_hand_true_ts = timeseries(traj.omega_hand_true, t);

% 真实欧拉角，仅用于观察和误差对比，单位 deg
eul_arm_true_ts   = timeseries(traj.eul_arm_true_deg, t);
eul_hand_true_ts  = timeseries(traj.eul_hand_true_deg, t);
eul_wrist_true_ts = timeseries(traj.eul_wrist_true_deg, t);

%% 生成预噪声

N = traj.N;
t = traj.t;

% 陀螺仪白噪声，单位 rad/s
gyro_noise_arm  = randn(N,3) .* imu.gyroNoiseStd_arm;
gyro_noise_hand = randn(N,3) .* imu.gyroNoiseStd_hand;

% 加速度计白噪声，单位 m/s^2
acc_noise_arm  = randn(N,3) .* imu.accNoiseStd_arm;
acc_noise_hand = randn(N,3) .* imu.accNoiseStd_hand;

gyro_noise_arm_ts  = timeseries(gyro_noise_arm, t);
gyro_noise_hand_ts = timeseries(gyro_noise_hand, t);

acc_noise_arm_ts  = timeseries(acc_noise_arm, t);
acc_noise_hand_ts = timeseries(acc_noise_hand, t);

%% 5. 送入 base workspace

assignin('base', 'cfg', cfg);
assignin('base', 'traj', traj);
assignin('base', 'imu', imu);

assignin('base', 'Ts', cfg.Ts);
assignin('base', 'Fs', cfg.Fs);
assignin('base', 'T', cfg.T);

assignin('base', 'q_arm_true_ts', q_arm_true_ts);
assignin('base', 'q_hand_true_ts', q_hand_true_ts);
assignin('base', 'q_wrist_true_ts', q_wrist_true_ts);

assignin('base', 'omega_arm_true_ts', omega_arm_true_ts);
assignin('base', 'omega_hand_true_ts', omega_hand_true_ts);

assignin('base', 'eul_arm_true_ts', eul_arm_true_ts);
assignin('base', 'eul_hand_true_ts', eul_hand_true_ts);
assignin('base', 'eul_wrist_true_ts', eul_wrist_true_ts);

assignin('base','gyro_noise_arm_ts',gyro_noise_arm_ts);
assignin('base','gyro_noise_hand_ts',gyro_noise_hand_ts);
assignin('base','acc_noise_arm_ts',acc_noise_arm_ts);
assignin('base','acc_noise_hand_ts',acc_noise_hand_ts);

%% 6. 简单检查

disp('初始化完成。');
disp('已生成以下 Simulink 输入变量：');
disp('  q_arm_true_ts');
disp('  q_hand_true_ts');
disp('  q_wrist_true_ts');
disp('  omega_arm_true_ts');
disp('  omega_hand_true_ts');
disp('  eul_wrist_true_ts');

figure;
plot(t, traj.eul_wrist_true_deg, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Wrist relative angle truth (deg)');
legend('roll','pitch','yaw');
title('True Wrist Relative Attitude');

figure;
plot(t, traj.omega_arm_true / deg, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Arm angular velocity (deg/s)');
legend('\omega_x','\omega_y','\omega_z');
title('True Arm Angular Velocity');

figure;
plot(t, traj.omega_hand_true / deg, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Hand angular velocity (deg/s)');
legend('\omega_x','\omega_y','\omega_z');
title('True Hand Angular Velocity');