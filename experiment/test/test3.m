clear; clc; close all;

%% 参数

Fs = 50;              % 模拟 50 Hz 输出
dt = 1/Fs;
T = 20;
tList = 0:dt:T;

buffer = uint8([]);

angleLog = [];
timeLog = [];

%% 图窗

figure('Name','Fake WIT Realtime Angle');
hRoll = animatedline('LineWidth',1.2);
hPitch = animatedline('LineWidth',1.2);
hYaw = animatedline('LineWidth',1.2);

grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Roll','Pitch','Yaw');
title('Realtime Parsed Angle from Fake WIT Packets');

%% 模拟实时数据流

tic;

for i = 1:length(tList)
    t = tList(i);

    % 1. 构造一个随时间变化的姿态角
    roll  = 20 * sin(0.8 * t);
    pitch = 15 * sin(0.5 * t + 0.4);
    yaw   = 30 * sin(0.3 * t + 0.8);

    angle_true = [roll, pitch, yaw];

    % 2. 按维特协议生成 angle 数据包 0x53
    pkt = wit_make_packet('angle', angle_true);

    % 3. 模拟真实串口不是每次完整到包
    % 这里故意随机切分数据
    if rand < 0.3
        cut = randi([1,10]);
        chunks = {pkt(1:cut), pkt(cut+1:end)};
    else
        chunks = {pkt};
    end

    % 4. 把"收到的数据"送入流解析器
    for c = 1:numel(chunks)
        [outs, buffer] = wit_parse_stream(buffer, chunks{c});

        for k = 1:numel(outs)
            out = outs{k};

            if strcmp(out.typeName, 'ANGLE')
                angle = out.angle_deg;

                angleLog = [angleLog; angle]; %#ok<AGROW>
                timeLog = [timeLog; t];       %#ok<AGROW>

                addpoints(hRoll,  t, angle(1));
                addpoints(hPitch, t, angle(2));
                addpoints(hYaw,   t, angle(3));

                drawnow limitrate;
            end
        end
    end

    % 5. 模拟真实时间流逝
    elapsed = toc;
    targetTime = t;
    if targetTime > elapsed
        pause(targetTime - elapsed);
    end
end

disp('假实时数据流测试完成。');