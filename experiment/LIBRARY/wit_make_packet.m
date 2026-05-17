function packet = wit_make_packet(typeName, values)
%WIT_MAKE_PACKET 生成模拟维特 WT-IMU 数据包
%
% typeName:
%   'acc'   values = [ax ay az]，单位 m/s^2
%   'gyro'  values = [wx wy wz]，单位 deg/s
%   'angle' values = [roll pitch yaw]，单位 deg
%   'quat'  values = [q0 q1 q2 q3]

typeName = lower(typeName);
values = double(values(:).');

switch typeName
    case 'acc'
        typeCode = hex2dec('51');
        raw = round(values / (16 * 9.81) * 32768);
        raw = [raw(1:3), 0];

    case 'gyro'
        typeCode = hex2dec('52');
        raw = round(values / 2000 * 32768);
        raw = [raw(1:3), 0];

    case 'angle'
        typeCode = hex2dec('53');
        raw = round(values / 180 * 32768);
        raw = [raw(1:3), 0];

    case 'quat'
        typeCode = hex2dec('59');
        raw = round(values * 32768);

    otherwise
        error('Unknown typeName: %s', typeName);
end

raw = max(min(raw, 32767), -32768);

packet = zeros(1, 11, 'uint8');
packet(1) = hex2dec('55');
packet(2) = typeCode;

idx = 3;
for i = 1:4
    [L, H] = int16_to_le(raw(i));
    packet(idx) = L;
    packet(idx+1) = H;
    idx = idx + 2;
end

packet(11) = uint8(mod(sum(uint16(packet(1:10))), 256));

end

function [L, H] = int16_to_le(x)
% int16 转低字节、高字节

x = round(x);

if x < 0
    u = x + 65536;
else
    u = x;
end

u = uint16(u);
L = uint8(bitand(u, 255));
H = uint8(bitshift(u, -8));
end