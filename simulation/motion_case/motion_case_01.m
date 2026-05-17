% motion_case_01.m
%
% Function: 预生成运动案例1
% 
% 四元数约定：
% q = [w x y z]，标量在前。
% q_arm_true   : 臂相对于世界的姿态
% q_wrist_true : 手相对于臂的相对姿态，即目标真值
% q_hand_true  : 手相对于世界的姿态
%
% 组合关系：
% q_hand_true = q_arm_true ⊗ q_wrist_true
%
% Author:    Chen Shuyi
% Date:      2026-5-12
function traj = motion_case_01(cfg)

if nargin < 1
    cfg = struct();
end

if ~isfield(cfg, 'Ts')
    cfg.Ts = 0.01;
end

if ~isfield(cfg, 'T')
    cfg.T = 20;
end

Ts = cfg.Ts;
T  = cfg.T;

t = (0:Ts:T)';
N = length(t);
deg = pi/180;

q_arm_true   = zeros(N,4);
q_wrist_true = zeros(N,4);
q_hand_true  = zeros(N,4);

eul_arm_true_deg   = zeros(N,3);
eul_wrist_true_deg = zeros(N,3);
eul_hand_true_deg  = zeros(N,3);

for k = 1:N
    tk = t(k);

    % 1. 手臂相对于世界的姿态：3自由度背景运动
    % 欧拉角顺序采用 ZYX，即 R = Rz(yaw) * Ry(pitch) * Rx(roll)
    arm_roll  = 15 * sin(0.50 * tk);   % deg
    arm_pitch = 20 * sin(0.35 * tk);   % deg
    arm_yaw   = 25 * sin(0.20 * tk);   % deg

    % 2. 手腕相对于手臂的姿态：第一版只做1自由度屈伸
    wrist_roll  = 0;
    wrist_pitch = 30 * sin(0.80 * tk); % deg，暂时把pitch理解为屈伸方向
    wrist_yaw   = 0;

    q_arm_true(k,:) = eulZYX_to_quat( ...
        arm_roll*deg, arm_pitch*deg, arm_yaw*deg);

    q_wrist_true(k,:) = eulZYX_to_quat( ...
        wrist_roll*deg, wrist_pitch*deg, wrist_yaw*deg);

    eul_arm_true_deg(k,:) = [arm_roll, arm_pitch, arm_yaw];
    eul_wrist_true_deg(k,:) = [wrist_roll, wrist_pitch, wrist_yaw];
end

% 保证四元数符号连续，避免 q 和 -q 跳变影响角速度反算
q_arm_true = quat_make_continuous(q_arm_true);
q_wrist_true = quat_make_continuous(q_wrist_true);

% 由 q_hand = q_arm ⊗ q_wrist 得到手部世界姿态
for k = 1:N
    q_hand_true(k,:) = quat_mul(q_arm_true(k,:), q_wrist_true(k,:));
end

q_hand_true = quat_make_continuous(q_hand_true);

% 由姿态序列反算陀螺仪真实角速度，单位 rad/s
% 这里角速度是在各自 IMU/刚体坐标系下表达的，符合陀螺仪输出习惯
omega_arm_true  = quat_sequence_to_body_omega(q_arm_true, Ts);
omega_hand_true = quat_sequence_to_body_omega(q_hand_true, Ts);
omega_wrist_rel_true = quat_sequence_to_body_omega(q_wrist_true, Ts);

% 手部欧拉角仅用于观察，不作为目标量
for k = 1:N
    eul_hand_true_deg(k,:) = quat_to_eulZYX(q_hand_true(k,:)) / deg;
end

traj.name = 'motion_case_01';
traj.description = 'Arm 3-DOF background motion + wrist 1-DOF relative flexion';

traj.Ts = Ts;
traj.T = T;
traj.t = t;
traj.N = N;

traj.q_arm_true = q_arm_true;
traj.q_wrist_true = q_wrist_true;
traj.q_hand_true = q_hand_true;

traj.omega_arm_true = omega_arm_true;
traj.omega_hand_true = omega_hand_true;
traj.omega_wrist_rel_true = omega_wrist_rel_true;

traj.eul_arm_true_deg = eul_arm_true_deg;
traj.eul_wrist_true_deg = eul_wrist_true_deg;
traj.eul_hand_true_deg = eul_hand_true_deg;

end
