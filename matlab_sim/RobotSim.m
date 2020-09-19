clear
clc
robot=importrobot('Leg.urdf');
show(robot);
axes.CameraPositionMode = 'auto';
%% Define the trajectory as a circle with a radius of 0.15
t = (0:0.2:20)';
count = length(t);
center = [0.0575 0 -0.4];
radius = 0.1;
theta = t*(2*pi/t(end));
points =(center + radius*[cos(theta) sin(theta) zeros(size(theta))])';
%% Draw the defined trajectory and inverse kinematics solution
hold on
plot3(points(1,:),points(2,:),points(3,:),'r')
%eeOffset = 0.01;
%eeBody = robotics.RigidBody('end_effector');%定义末端执行器，并再后面添加到机器人的Link5上
%setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
%addBody(robot,eeBody,'puma_link_5');
ik = robotics.InverseKinematics('RigidBodyTree',robot);%逆解解算器
weights = [0.1 0.1 0 0 0 0]; %权重
qInitial = robot.homeConfiguration;
%%
%通过点的轨迹循环来跟踪圆。调用每个点的ik对象以生成实现末端位置的关节配置，存储要稍后使用的逆解结果。
for i = 1:size(points,2)
% Solve for the configuration satisfying the desired end effector
tform = rpy2tr(136,-180,-180);%姿态齐次矩阵
tform = trvec2tform(points(:,i)')*tform ;%末端位姿齐次矩阵
qSol = ik('fl_foot',tform,weights,qInitial);%求解各关节角度
% Start from prior solution
qInitial = qSol;
end
%% 动画显示
title('robot move follow the trajectory')
hold on
axis([-0.8 0.8 -0.8 0.85 0 1.3]);
for i = 1:size(points,2)
show(robot,qSol','PreservePlot',false);%false改为true时，留下重影。
pause(0.3)
plot3(points(1,i),points(2,i),points(3,i),'.','LineWidth',1);
end