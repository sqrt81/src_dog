% 本脚本记录机器人左前腿的质量、转动惯量、质心位置等动力学参数

syms m_hip m_thigh m_shin; % 髋关节、大腿、小腿质量

% 上述三者的质心位置，以各坐标轴正向为正
syms com_hip_x com_hip_y;
syms com_thigh_y com_thigh_z;
syms com_shin_z;

% 三个元件的转动惯量（相对质心）
syms I1xx I1xy I1yy I1zz; % hip
syms I2xx I2yy I2yz I2zz; % thigh
syms I3xx I3yy I3zz; % shin

CoM_hip = [
    com_hip_x;  com_hip_y;      0
    ];
CoM_thigh = [
    0;          com_thigh_y;    com_thigh_z
    ];
CoM_shin = [
    0;          0;              com_shin_z
    ];

I1sum = (I1xx + I1yy + I1zz) / 2;
I2sum = (I2xx + I2yy + I2zz) / 2;
I3sum = (I3xx + I3yy + I3zz) / 2;

% 二阶矩张量（相对参考系原点）（不是惯量张量！）
I_hip = [
    I1sum - I1xx    -I1xy           0
    -I1xy           I1sum - I1yy    0
    0               0               I1sum - I1zz
    ] + m_hip * CoM_hip * transpose(CoM_hip);
I_thigh = [
    I2sum - I2xx    0               0
    0               I2sum - I2yy    -I2yz
    0               -I2yz           I2sum - I2zz
    ] + m_thigh * CoM_thigh * transpose(CoM_thigh);
I_shin = [
    I3sum - I3xx    0               0
    0               I3sum - I3yy    0
    0               0               I3sum - I3zz
    ] + m_shin * CoM_shin * transpose(CoM_shin);

% M 表示齐次质量矩阵，可以用来计算动能
% Ek = trace(Vt * M * transpose(Vt)) / 2
% 其中 Vt = dT/dt，是齐次变换矩阵的时间变化率
M_hip = [
    I_hip                               CoM_hip * m_hip
    transpose(CoM_hip) * m_hip          m_hip
    ];
M_thigh = [
    I_thigh                             CoM_thigh * m_thigh
    transpose(CoM_thigh) * m_thigh      m_thigh
    ];
M_shin = [
    I_shin                              CoM_shin * m_shin
    transpose(CoM_shin) * m_shin        m_shin
    ];
