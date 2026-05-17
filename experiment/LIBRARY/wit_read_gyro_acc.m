function [gyroList, accList, buffer] = wit_read_gyro_acc(s, buffer)
%WIT_READ_GYRO_ACC 从维特 IMU 串口中读取角速度和加速度数据
%
% 输入：
%   s      : serialport 串口对象
%   buffer : 上一轮未解析完的残余字节
%
% 输出：
%   gyroList : N×3 角速度数据，单位 deg/s
%   accList  : M×3 加速度数据，单位 m/s^2
%   buffer   : 本轮解析后剩余未完成的数据
%
% 注意：
%   1. 上位机需要勾选"角速度"和"加速度"输出。
%   2. 本函数依赖 wit_parse_stream.m。
%   3. gyroList 和 accList 的行数不一定相同，因为串口数据到达不一定完全同步。

gyroList = [];
accList = [];

n = s.NumBytesAvailable;

if n <= 0
    return;
end

newBytes = read(s, n, "uint8");
[outs, buffer] = wit_parse_stream(buffer, newBytes);

for k = 1:numel(outs)
    out = outs{k};

    switch out.typeName
        case 'GYRO'
            gyroList = [gyroList; out.gyro_deg_s]; %#ok<AGROW>

        case 'ACC'
            accList = [accList; out.acc]; %#ok<AGROW>
    end
end

end