%% check_gyro_packet_rate.m
clear; clc;

com = "COM5";
baud = 115200;

s = serialport(com, baud, "Timeout", 0.05);
flush(s);

buffer = uint8([]);

count = 0;
gyroAll = [];

recordTime = 10;
t0 = tic;

while toc(t0) < recordTime
    n = s.NumBytesAvailable;

    if n > 0
        newBytes = read(s, n, "uint8");
        [outs, buffer] = wit_parse_stream(buffer, newBytes);

        for k = 1:numel(outs)
            out = outs{k};

            if strcmp(out.typeName, 'GYRO')
                count = count + 1;
                gyroAll = [gyroAll; out.gyro_deg_s]; %#ok<AGROW>
            end
        end
    end

    pause(0.001);
end

clear s;

fprintf("总GYRO包数 = %d\n", count);
fprintf("平均频率 = %.2f Hz\n", count / recordTime);

fprintf("gyro mean = %.6f %.6f %.6f deg/s\n", mean(gyroAll,1));
fprintf("gyro std  = %.6f %.6f %.6f deg/s\n", std(gyroAll,0,1));
fprintf("gyro min  = %.6f %.6f %.6f deg/s\n", min(gyroAll,[],1));
fprintf("gyro max  = %.6f %.6f %.6f deg/s\n", max(gyroAll,[],1));