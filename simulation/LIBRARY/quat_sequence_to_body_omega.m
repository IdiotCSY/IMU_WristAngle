function omega = quat_sequence_to_body_omega(Q, Ts)
% 由姿态序列反算体坐标系下角速度
% q_next = q_current ⊗ dq_body
% dq_body = q_current^{-1} ⊗ q_next

N = size(Q,1);
omega = zeros(N,3);

for k = 1:N-1
    dq = quat_mul(quat_conj(Q(k,:)), Q(k+1,:));

    if dq(1) < 0
        dq = -dq;
    end

    rotvec = quat_to_rotvec(dq);
    omega(k,:) = rotvec / Ts;
end

omega(N,:) = omega(N-1,:);
end