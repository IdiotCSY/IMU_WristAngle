clear; clc;

%% 1. 生成几包假数据

pkt1 = wit_make_packet('angle', [10, 20, -30]);
pkt2 = wit_make_packet('quat',  [0.9238795, 0, 0.3826834, 0]);
pkt3 = wit_make_packet('gyro',  [1.2, -0.5, 3.0]);

% 模拟真实串口字节流：前面加杂散字节，中间连续拼接
stream = uint8([99 88 77, pkt1, pkt2, pkt3]);

%% 2. 模拟分段读取

buffer = uint8([]);

chunks = {
    stream(1:7)
    stream(8:20)
    stream(21:end)
};

for i = 1:numel(chunks)

    [outs, buffer] = wit_parse_stream(buffer, chunks{i});

    fprintf('\n===== Chunk %d =====\n', i);
    fprintf('剩余 buffer 长度: %d\n', numel(buffer));

    for k = 1:numel(outs)
        out = outs{k};

        switch out.typeName
            case 'ANGLE'
                fprintf('Angle = %.3f %.3f %.3f deg\n', out.angle_deg);

            case 'QUAT'
                fprintf('Quat  = %.6f %.6f %.6f %.6f\n', out.quat);

            case 'GYRO'
                fprintf('Gyro  = %.3f %.3f %.3f deg/s\n', out.gyro_deg_s);

            case 'ACC'
                fprintf('Acc   = %.3f %.3f %.3f m/s^2\n', out.acc);

            otherwise
                disp(out);
        end
    end
end