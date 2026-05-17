clear; clc; close all;

com = "COM5";
baud = 115200;

s = serialport(com, baud, "Timeout", 1);
flush(s);

buffer = uint8([]);

%% 建立3D图形

fig = figure('Name','WT-IMU63 3D Attitude','Color','w');
ax = axes('Parent',fig);

axis(ax, 'equal');
grid(ax, 'on');
view(ax, 3);

xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');

xlim(ax, [-0.15 0.15]);
ylim(ax, [-0.15 0.15]);
zlim(ax, [-0.15 0.15]);

hold(ax, 'on');

% 世界坐标轴
plot3(ax, [0 0.12], [0 0], [0 0], 'r', 'LineWidth', 1.5);
plot3(ax, [0 0], [0 0.12], [0 0], 'g', 'LineWidth', 1.5);
plot3(ax, [0 0], [0 0], [0 0.12], 'b', 'LineWidth', 1.5);

text(0.125,0,0,'X');
text(0,0.125,0,'Y');
text(0,0,0.125,'Z');

% IMU刚体变换组
imuT = hgtransform('Parent', ax);

% 画一个简化IMU板子
[V,F] = make_box([0.08, 0.05, 0.01]);
patch('Vertices', V, 'Faces', F, ...
    'FaceColor', [0.7 0.8 1.0], ...
    'FaceAlpha', 0.85, ...
    'EdgeColor', [0.2 0.2 0.2], ...
    'Parent', imuT);

% IMU自身坐标轴
L = 0.08;
line([0 L], [0 0], [0 0], 'Color','r', 'LineWidth',2, 'Parent',imuT);
line([0 0], [0 L], [0 0], 'Color','g', 'LineWidth',2, 'Parent',imuT);
line([0 0], [0 0], [0 L], 'Color','b', 'LineWidth',2, 'Parent',imuT);

title(ax, 'Realtime WT-IMU63 Quaternion Attitude');

disp("开始3D显示，关闭图窗停止。");

%% 实时读取并更新3D姿态

lastDraw = tic;
drawInterval = 0.02;   % 约50Hz刷新上限

while ishandle(fig)
    n = s.NumBytesAvailable;

    if n > 0
        newBytes = read(s, n, "uint8");
        [outs, buffer] = wit_parse_stream(buffer, newBytes);

        for k = 1:numel(outs)
            out = outs{k};

            if strcmp(out.typeName, 'QUAT')
                q = out.quat;          % [q0 q1 q2 q3]
                q = q / norm(q);

                if toc(lastDraw) > drawInterval
                    R = quat_to_rotm_scalar_first(q);

                    % 如果发现转动方向明显反了，把下一行改成 R = R';
                    T = eye(4);
                    T(1:3,1:3) = R;

                    set(imuT, 'Matrix', T);

                    drawnow limitrate;
                    lastDraw = tic;
                end
            end
        end
    end

    pause(0.002);
end

clear s;
disp("3D显示结束。");

%% ===== 局部函数 =====

function [V,F] = make_box(dim)
% 生成中心在原点的长方体
% dim = [Lx Ly Lz]

Lx = dim(1);
Ly = dim(2);
Lz = dim(3);

x = Lx/2;
y = Ly/2;
z = Lz/2;

V = [
    -x -y -z
     x -y -z
     x  y -z
    -x  y -z
    -x -y  z
     x -y  z
     x  y  z
    -x  y  z
];

F = [
    1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
];
end

function R = quat_to_rotm_scalar_first(q)
% q = [q0 q1 q2 q3] = [w x y z]

q = q(:);
q = q / norm(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R = zeros(3,3);

R(1,1) = 1 - 2*(y^2 + z^2);
R(1,2) = 2*(x*y - w*z);
R(1,3) = 2*(x*z + w*y);

R(2,1) = 2*(x*y + w*z);
R(2,2) = 1 - 2*(x^2 + z^2);
R(2,3) = 2*(y*z - w*x);

R(3,1) = 2*(x*z - w*y);
R(3,2) = 2*(y*z + w*x);
R(3,3) = 1 - 2*(x^2 + y^2);
end