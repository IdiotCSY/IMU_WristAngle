function q = quat_normalize(q)
%QUAT_NORMALIZE 四元数归一化
%
% 输入：
%   q : 四元数 [w x y z]
%
% 输出：
%   q : 单位四元数

q = q(:).';

n = norm(q);

if n < 1e-12
    q = [1 0 0 0];
else
    q = q / n;
end
end