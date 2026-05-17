function eul = quat_to_eulZYX(q)
% 输出 [roll pitch yaw]，单位 rad

q = q / norm(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R11 = 1 - 2*(y^2 + z^2);
R21 = 2*(x*y + w*z);
R31 = 2*(x*z - w*y);
R32 = 2*(y*z + w*x);
R33 = 1 - 2*(x^2 + y^2);

pitch = asin(-R31);
roll  = atan2(R32, R33);
yaw   = atan2(R21, R11);

eul = [roll, pitch, yaw];
end
