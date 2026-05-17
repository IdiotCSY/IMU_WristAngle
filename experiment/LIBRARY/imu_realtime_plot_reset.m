function P = imu_realtime_plot_reset(P)
%IMU_REALTIME_PLOT_RESET 清空实时曲线，并复位按键状态
% 
% ================= 使用说明 =================
% 功能：
%   清空实时显示中的曲线数据，并重置绘图状态。
%   一般在主脚本检测到按键 z 后调用，用于重新开始记录曲线。
%
% 输入：
%   P : 绘图状态结构体，由 imu_realtime_plot_init 返回。
%
% 输出：
%   P : 重置后的绘图状态结构体。
%
% 典型用法：
%   if P.fig.UserData.zeroRequested
%       % 1. 主脚本中先完成算法零位处理
%       % 例如：
%       %   qHand = [1 0 0 0];
%       %   qArm  = [1 0 0 0];
%       %   qRel0 = qRel;
%
%       % 2. 然后清空绘图数据
%       P = imu_realtime_plot_reset(P);
%
%       % 3. 重新计时
%       tStart = tic;
%   end
%
% 注意事项：
%   1. 本函数只清空图像缓存，不会自动修改算法变量。
%      例如 qHand、qArm、qRel0、qHand0、qArm0 等必须在主脚本中处理。
%
%   2. 本函数会将：
%          P.fig.UserData.zeroRequested = false
%          P.fig.UserData.zeroDone = false
%      如果主脚本需要记录"已完成零位"，可以在调用后自行设置。
%
%   3. 本函数不会关闭图窗，也不会释放串口。
%      串口释放应在主脚本结束时处理，例如：
%          clear sHand sArm
%
%   4. 本函数适合用于"按 z 重新置零"或"重新开始一次测试"的场景。
%      不建议在正常循环中频繁调用，否则曲线会一直被清空。
%
%   5. 如果只想重新定义算法零位，但不想清空曲线，则不要调用本函数。
% ============================================

P.tLog = [];
P.relEulerLog = [];
P.thetaLog = [];

if ishandle(P.hRelRoll)
    set(P.hRelRoll,  'XData', nan, 'YData', nan);
    set(P.hRelPitch, 'XData', nan, 'YData', nan);
    set(P.hRelYaw,   'XData', nan, 'YData', nan);
    set(P.hTheta,    'XData', nan, 'YData', nan);
end

if ishandle(P.fig)
    P.fig.UserData.zeroRequested = false;
    P.fig.UserData.zeroDone = false;
end

P.lastDraw = tic;
P.lastXlimUpdate = tic;

end