% 根据末端位置确定关节角度

LegTransform;

syms xt yt zt;

% 以下两个变量用于确定机器人的姿态模式
hip_out = true;
knee_out = false;

% 目标点相对左前髋关节（fl_hip_x）的坐标
x_h = xt - hip_x - hip_len_x;
y_h = yt - hip_y;

angle_11 = acos(hip_len_y / sqrt(zt * zt + y_h * y_h));
angle_12 = atan2(zt, y_h);

if hip_out
    q1_eval = angle_11 + angle_12;
else
    q1_eval = - angle_11 + angle_12;
end

w_h = sin(q1_eval) * y_h - cos(q1_eval) * zt;
leg_l_2 = x_h * x_h + w_h * w_h;
angle_3 = acos((leg_l_2 - thigh_offset_z * thigh_offset_z - shin_offset_z * shin_offset_z) ...
 / 2 * thigh_offset_z * shin_offset_z);

if knee_out
    q3_eval = angle_3
else
    q3_eval = - angle_3
end

q2 = acos(w_h / sqrt(leg_l_2)) - asin(shin_offset_z * sin(q3_eval) / sqrt(leg_l_2))

