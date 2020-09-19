% 本脚本用于记录机器人左前腿的长度、定义相关变量
% 可以用于机器人运动学推导。

% 左前腿长度参数（胯宽、大腿长、小腿长），以坐标轴正向为正
syms hip_len_x hip_len_y thigh_offset_z shin_offset_z; 

syms hip_x hip_y; % 从躯干中心点到左前腿的位移
syms q1 q2 q3; % 关节角度

c1 = cos(q1);
s1 = sin(q1);
c2 = cos(q2);
s2 = sin(q2);
c3 = cos(q3);
s3 = sin(q3);

% 从胯到躯干的齐次变换矩阵
T1 = [
    1   0   0   hip_x
    0   c1  -s1 hip_y
    0   s1  c1  0
    0   0   0   1
    ];
% T1 * 点在胯参考系下坐标 = 点在躯干参考系下坐标

% 从大腿到胯的齐次变换矩阵
T2 = [
    c2  0   s2  hip_len_x
    0   1   0   hip_len_y
    -s2 0   c2  0
    0   0   0   1
    ];

% 从小腿到大腿的齐次变换矩阵
T3 = [
    c3  0   s3  0
    0   1   0   0
    -s3 0   c3  thigh_offset_z
    0   0   0   1
    ];

% 末端在小腿参考系中的坐标（齐次）
FootLocalPos = [
    0
    0
    shin_offset_z
    1
    ];
