function out = wit_parse_packet(packet)
%WIT_PARSE_PACKET 解析维特 WT-IMU 数据包
% 输入 packet: 1x11 uint8
% 支持：
%   0x51 加速度
%   0x52 角速度
%   0x53 角度
%   0x59 四元数

packet = uint8(packet(:).');

if numel(packet) ~= 11
    error('WIT packet must be 11 bytes.');
end

out = struct();
out.valid = false;
out.type = packet(2);
out.typeName = 'UNKNOWN';

% 检查帧头
if packet(1) ~= hex2dec('55')
    return;
end

% 检查校验和
sum_calc = uint8(mod(sum(uint16(packet(1:10))), 256));
if sum_calc ~= packet(11)
    return;
end

out.valid = true;

% 读取四个 int16 数据
d1 = read_int16_le(packet(3), packet(4));
d2 = read_int16_le(packet(5), packet(6));
d3 = read_int16_le(packet(7), packet(8));
d4 = read_int16_le(packet(9), packet(10));

switch packet(2)
    case hex2dec('51')
        out.typeName = 'ACC';
        % 单位 m/s^2
        out.acc = [d1, d2, d3] / 32768 * 16 * 9.81;
        out.temp_raw = d4;

    case hex2dec('52')
        out.typeName = 'GYRO';
        % 单位 deg/s
        out.gyro_deg_s = [d1, d2, d3] / 32768 * 2000;
        out.vol_raw = d4;

    case hex2dec('53')
        out.typeName = 'ANGLE';
        % 单位 deg
        out.angle_deg = [d1, d2, d3] / 32768 * 180;
        out.version_raw = d4;

    case hex2dec('59')
        out.typeName = 'QUAT';
        % 四元数 [q0 q1 q2 q3]
        out.quat = [d1, d2, d3, d4] / 32768;

    otherwise
        out.raw = [d1, d2, d3, d4];
end

end

function val = read_int16_le(lowByte, highByte)
% 低字节在前，高字节在后，转换为有符号 int16

u = double(uint16(lowByte)) + double(bitshift(uint16(highByte), 8));

if u >= 32768
    val = u - 65536;
else
    val = u;
end
end