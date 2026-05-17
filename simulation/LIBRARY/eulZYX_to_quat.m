function q = eulZYX_to_quat(roll, pitch, yaw)
% R = Rz(yaw) * Ry(pitch) * Rx(roll)

qx = [cos(roll/2),  sin(roll/2), 0, 0];
qy = [cos(pitch/2), 0, sin(pitch/2), 0];
qz = [cos(yaw/2),   0, 0, sin(yaw/2)];

q = quat_mul(qz, quat_mul(qy, qx));
q = q / norm(q);
end