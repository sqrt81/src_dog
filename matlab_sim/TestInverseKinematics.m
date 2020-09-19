% 验证逆运动学是否正确

clear;
clc;

InverseKinematics;

% 随机设定目标，注意目标要在可达范围内
target = (rand(1, 3) - 0.5) * 0.1 + [0.283  0.118   -0.3];

sym_list = [
    hip_len_x hip_len_y ...
    thigh_offset_z shin_offset_z ...
    hip_x hip_y ...
    xt yt zt
    ];
sub_list = [
    0.053   0.0575  ...
    -0.2    -0.2    ...
    0.23    0.0605  ...
    target
    ];

q1_value = eval(subs(q1_eval, sym_list, sub_list));
q2_value = eval(subs(q2_eval, sym_list, sub_list));
q3_value = eval(subs(q3_eval, sym_list, sub_list));

foot_pos = T1 * T2 * T3 * FootLocalPos;
foot_pos_value = eval(subs(foot_pos(1:3), [sym_list q1 q2 q3], ...
                           [sub_list q1_value q2_value q3_value]));

disp("Set target to:");
disp(target);
disp("Computed joint pos:");
disp([q1_value q2_value q3_value]);
disp("Foot pos:");
disp(transpose(foot_pos_value));

