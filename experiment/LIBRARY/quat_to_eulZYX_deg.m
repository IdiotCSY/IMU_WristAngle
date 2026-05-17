function eulDeg = quat_to_eulZYX_deg(q)
%QUAT_TO_EULZYX_DEG 四元数转 ZYX 欧拉角
%
% 输入：
%   q : 四元数 [w x y z]
%
% 输出：
%   eulDeg : [roll pitch yaw]，单位 deg
%
% 说明：
%   采用 ZYX 顺序：
%       R = Rz(yaw) * Ry(pitch) * Rx(roll)

q = quat_normalize(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R11 = 1 - 2*(y^2 + z^2);
R21 = 2*(x*y + w*z);
R31 = 2*(x*z - w*y);
R32 = 2*(y*z + w*x);
R33 = 1 - 2*(x^2 + y^2);

pitch = asin(max(min(-R31, 1), -1));
roll  = atan2(R32, R33);
yaw   = atan2(R21, R11);

eulDeg = [roll, pitch, yaw] * 180/pi;
eulDeg = mod(eulDeg + 180, 360) - 180;
end