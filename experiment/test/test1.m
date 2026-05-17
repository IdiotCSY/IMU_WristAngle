clear; clc;

paths = setup_project_paths();

%% 1. 测试角度包 0x53

angle_true = [10, 20, -30];   % roll pitch yaw, deg

pkt_angle = wit_make_packet('angle', angle_true);
out_angle = wit_parse_packet(pkt_angle);

disp('===== Angle Packet =====');
disp(pkt_angle);
disp(out_angle);

fprintf('True angle:   %.3f %.3f %.3f deg\n', angle_true);
fprintf('Parsed angle: %.3f %.3f %.3f deg\n', out_angle.angle_deg);

%% 2. 测试四元数包 0x59

q_true = [0.9238795, 0, 0.3826834, 0];  % 绕Y轴约45度

pkt_quat = wit_make_packet('quat', q_true);
out_quat = wit_parse_packet(pkt_quat);

disp('===== Quaternion Packet =====');
disp(pkt_quat);
disp(out_quat);

fprintf('True quat:   %.6f %.6f %.6f %.6f\n', q_true);
fprintf('Parsed quat: %.6f %.6f %.6f %.6f\n', out_quat.quat);