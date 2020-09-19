LegTransform;

FootPos = (T1 * T2 * T3 * FootLocalPos);
FootPos = FootPos(1:3);
disp("End Effector Position:");
disp(FootPos);

J = jacobian(FootPos, [p1, p2, p3]);
disp("End Effector Jacobian:");
disp(J);
