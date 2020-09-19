% 计算动力学参数：H、C、G，满足：
% tau = H * ddq + C * dq + G * g，
% 其中，H = H(q)，C = C(q, vq)，G = G(q),
% 且满足关系：H = transpose(H)， dH = C + transpose(C)。

LegDynamicParams;

% H
H = jacobian(torq, acc);

% C
C = jacobian(torq, vel) / 2;

% G
G = jacobian(torq, gravity(1:3));

