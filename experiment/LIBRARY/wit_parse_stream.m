function [outputs, buffer] = wit_parse_stream(buffer, newBytes)
%WIT_PARSE_STREAM 从连续字节流中解析维特WT-IMU数据包
%
% 输入：
%   buffer   : 上一次未解析完的残余字节
%   newBytes : 本次新收到的字节
%
% 输出：
%   outputs : cell数组，每个cell里是一个解析后的结构体
%   buffer  : 剩余未解析字节，留到下次继续用

buffer = uint8([buffer(:); newBytes(:)]);
outputs = {};
idxOut = 1;

while numel(buffer) >= 11

    % 1. 找帧头 0x55
    headIdx = find(buffer == hex2dec('55'), 1, 'first');

    if isempty(headIdx)
        buffer = uint8([]);
        return;
    end

    % 2. 丢掉帧头前面的杂散字节
    if headIdx > 1
        buffer = buffer(headIdx:end);
    end

    % 3. 如果剩余不足 11 字节，等待下次读取
    if numel(buffer) < 11
        return;
    end

    % 4. 取出一帧
    packet = buffer(1:11).';

    % 5. 解析
    out = wit_parse_packet(packet);

    if out.valid
        outputs{idxOut} = out; %#ok<AGROW>
        idxOut = idxOut + 1;

        % 成功解析，删除这一帧
        buffer = buffer(12:end);
    else
        % 校验失败，可能错位，丢掉当前0x55继续找下一个
        buffer = buffer(2:end);
    end
end

end