function qc = quat_conj(q)
%QUAT_CONJ 四元数共轭
%
% 输入：
%   q : 四元数 [w x y z]
%
% 输出：
%   qc : 共轭四元数 [w -x -y -z]

q = q(:).';
qc = [q(1), -q(2), -q(3), -q(4)];
end