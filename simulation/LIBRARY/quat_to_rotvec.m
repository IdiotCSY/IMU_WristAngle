function r = quat_to_rotvec(q)
% 四元数转旋转向量，输出单位 rad

q = q / norm(q);

if q(1) < 0
    q = -q;
end

v_norm = norm(q(2:4));
angle = 2 * atan2(v_norm, q(1));

if v_norm < 1e-12
    r = [0, 0, 0];
else
    axis = q(2:4) / v_norm;
    r = angle * axis;
end
end