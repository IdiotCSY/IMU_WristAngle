function Q = quat_make_continuous(Q)
% 避免相邻四元数符号突然翻转

for k = 2:size(Q,1)
    if dot(Q(k,:), Q(k-1,:)) < 0
        Q(k,:) = -Q(k,:);
    end
end
end
