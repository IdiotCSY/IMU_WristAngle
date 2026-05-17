clear; clc; close all;

com = "COM5";      % 改成要测的 IMU
baud = 115200;

s = serialport(com, baud, "Timeout", 0.05);
flush(s);

buffer = uint8([]);

figure('Name','One IMU Gyro Output','Color','w');
hold on; grid on;

hX = plot(nan, nan, 'r', 'LineWidth', 1.2);
hY = plot(nan, nan, 'g', 'LineWidth', 1.2);
hZ = plot(nan, nan, 'b', 'LineWidth', 1.2);

legend('\omega_x','\omega_y','\omega_z');
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
title('Realtime Gyro Output');

tLog = [];
gyroLog = [];

t0 = tic;
showWindow = 8;
maxPoints = 1000;

disp("开始画角速度。关闭图窗停止。");

while ishandle(gcf)

    n = s.NumBytesAvailable;

    if n > 0
        newBytes = read(s, n, "uint8");
        [outs, buffer] = wit_parse_stream(buffer, newBytes);

        for k = 1:numel(outs)
            out = outs{k};

            if strcmp(out.typeName, 'GYRO')
                tNow = toc(t0);

                tLog = [tLog; tNow];
                gyroLog = [gyroLog; out.gyro_deg_s];

                if numel(tLog) > maxPoints
                    tLog = tLog(end-maxPoints+1:end);
                    gyroLog = gyroLog(end-maxPoints+1:end,:);
                end

                set(hX, 'XData', tLog, 'YData', gyroLog(:,1));
                set(hY, 'XData', tLog, 'YData', gyroLog(:,2));
                set(hZ, 'XData', tLog, 'YData', gyroLog(:,3));

                xlim([max(0,tNow-showWindow), max(showWindow,tNow)]);
                drawnow limitrate;
            end
        end
    end

    pause(0.001);
end

clear s;
disp("结束。");