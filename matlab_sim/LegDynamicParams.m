LegTransform;
LegInertial;

syms g_x g_y g_z; % gravitational field
syms vq1 vq2 vq3; % vq = dq/dt
syms aq1 aq2 aq3; % aq = dvq/dt

gravity = [
    g_x;    g_y;    g_z;    0
    ];

pos = [
    q1;     q2;     q3
    ];
vel = [
    vq1;    vq2;    vq3
    ];
acc = [
    aq1;    aq2;    aq3
    ];

T10 = T1;
T20 = T10 * T2;
T30 = T20 * T3;

dT11 = diff(T10, q1);
dT12 = diff(T10, q2);
dT13 = diff(T10, q3);
dT21 = diff(T20, q1);
dT22 = diff(T20, q2);
dT23 = diff(T20, q3);
dT31 = diff(T30, q1);
dT32 = diff(T30, q2);
dT33 = diff(T30, q3);

dT1 = dT11 * vq1 + dT12 * vq2 + dT13 * vq3;
dT2 = dT21 * vq1 + dT22 * vq2 + dT23 * vq3;
dT3 = dT31 * vq1 + dT32 * vq2 + dT33 * vq3;

Ek_1 = trace(dT1 * M_hip * transpose(dT1)) / 2;
Ek_2 = trace(dT2 * M_thigh * transpose(dT2)) / 2;
Ek_3 = trace(dT3 * M_shin * transpose(dT3)) / 2;

Ep_1 = - transpose(gravity) * T10 * M_hip   * [0;   0;  0;  1];
Ep_2 = - transpose(gravity) * T20 * M_thigh * [0;   0;  0;  1];
Ep_3 = - transpose(gravity) * T30 * M_shin  * [0;   0;  0;  1];

L = Ek_1 + Ek_2 + Ek_3 - Ep_1 - Ep_2 - Ep_3;

L_vq = jacobian(L, vel);
torq = ...
transpose(jacobian(L_vq, pos) * vel + jacobian(L_vq, vel) * acc) ...
- jacobian(L, pos);


