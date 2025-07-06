% 七轴机械臂逆运动学动画演示
clc; clear; close all;

% 1. 创建七轴机械臂模型
robot = create_robot();

% 2. 设置初始关节角和目标关节角（可替换为逆解结果）
q_init = [0 0 0 0 0 0 0]; % 初始角度（弧度）
%q_goal = [0.0 1.57 0 1.57 0 -1.57 0]; % 目标角度（弧度），可替换为逆解结果

%q_goal = [-0.068468, 1.552635, 0.086558, 1.571277, -0.017999, -1.569684, -0.018108];
%q_goal = [1.065200, 0.677774, -1.562480, -0.515720, -1.102202, -1.753814, -0.331867];
%q_goal = [-0.423903, 1.567427, 0.426993, 1.571264, -0.003002, -1.569876, -0.003072];
%q_goal = [1.571072, -0.003710, -1.570526, -1.571263, -0.079916, -0.003257, 0.079647];
q_goal = [-0.148986, 1.561645, 0.158092, 1.571273, -0.008863, -1.569816, -0.009122];
% 3. 插值生成轨迹
steps = 50;
q_traj = zeros(steps, 7);
for i = 1:steps
    q_traj(i, :) = q_init + (q_goal - q_init) * (i-1)/(steps-1);
end
T= robot. fkine(q_goal);
T
% 4. 动画演示
figure(1);
for i = 1:steps
    robot.plot(q_traj(i, :));%, 'workspace', [-0.5 0.5 -0.5 0.5 0 1])
    title('七轴机械臂运动轨迹');
    drawnow;
    pause(0.05);
end

%% 机械臂模型函数
function robot = create_robot()
    % 使用SerialLink创建机械臂模型
    L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0, 'modified');
    L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2, 'modified');
    L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'modified');
    L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
    L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
    L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
    L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
    for i = 1:7
        L(i).qlim = [-pi, pi];
    end
    robot = SerialLink(L, 'name', '7-DOF Robot');
    robot.base = transl(0, 0, 0.1299);
end