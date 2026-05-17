function [axis, angleDeg] = quat_to_axis_angle_deg(q)
%QUAT_TO_AXIS_ANGLE_DEG 四元数转轴角
%
% 输入：
%   q : 四元数 [w x y z]
%
% 输出：
%   axis     : 旋转轴单位向量 [ux uy uz]
%   angleDeg : 旋转角 theta，单位 deg，范围约为 [0, 180]

q = quat_normalize(q);

% q 和 -q 表示同一个姿态，统一到 w >= 0，避免角度跳变
if q(1) < 0
    q = -q;
end

w = q(1);
v = q(2:4);

vNorm = norm(v);
angle = 2 * atan2(vNorm, w);

if vNorm < 1e-8
    axis = [0 0 0];
    angleDeg = 0;
else
    axis = v / vNorm;
    angleDeg = angle * 180/pi;
end

angleDeg = min(max(angleDeg, 0), 180);
end