function q = quat_mul(q1, q2)
%QUAT_MUL 四元数乘法 q = q1 ⊗ q2
%
% 输入：
%   q1, q2 : 四元数，格式 [w x y z]
%
% 输出：
%   q : 乘积四元数，格式 [w x y z]

q1 = q1(:).';
q2 = q2(:).';

s1 = q1(1);
v1 = q1(2:4);

s2 = q2(1);
v2 = q2(2:4);

s = s1*s2 - dot(v1, v2);
v = s1*v2 + s2*v1 + cross(v1, v2);

q = [s, v];
end