% 创建7自由度机械臂的DH参数模型
% 注意：使用Robotics Toolbox for MATLAB

% 定义各连杆参数（根据提供的DH参数表）
L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0, 'modified');
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2, 'modified');
L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'modified');
L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');

% 设置关节限制
L(1).qlim = [-pi   pi];
L(2).qlim = [-pi/8 pi];
L(3).qlim = [-pi/2 pi/2];
L(4).qlim = [-pi/2 pi/2];
L(5).qlim = [-pi   pi/2];
L(6).qlim = [-pi/2 pi/2];
L(7).qlim = [-pi/2 pi/2];

% 创建串联机械臂对象
seven_dof = SerialLink(L, 'name', '7-dof');

% 设置机械臂基坐标位置
seven_dof.base = transl(0, 0, 0.1299);

% 显示机械臂结构和DH参数
disp('机械臂DH参数表：');
seven_dof.display();

% 生成工作空间点云
num = 30000; % 点数
P = zeros(num, 3); % 预分配内存

for i = 1:num
    % 生成随机关节角度
    q1 = L(1).qlim(1) + rand() * (L(1).qlim(2) - L(1).qlim(1));
    q2 = L(2).qlim(1) + rand() * (L(2).qlim(2) - L(2).qlim(1));
    q3 = L(3).qlim(1) + rand() * (L(3).qlim(2) - L(3).qlim(1));
    q4 = L(4).qlim(1) + rand() * (L(4).qlim(2) - L(4).qlim(1));
    q5 = L(5).qlim(1) + rand() * (L(5).qlim(2) - L(5).qlim(1));
    q6 = L(6).qlim(1) + rand() * (L(6).qlim(2) - L(6).qlim(1));
    q7 = L(7).qlim(1) + rand() * (L(7).qlim(2) - L(7).qlim(1));
    
    q = [q1 q2 q3 q4 q5 q6 q7];
    
    % 计算正向运动学
    T = seven_dof.fkine(q);
    
    % 提取位置
    P(i, :) = transl(T)';

    % seven_dof.plot (q);
    % [x,y,z]= transl(T);
    % plot3(x,y,z,'b','markersize',1);
end

% 绘制工作空间
figure;
plot3(P(:, 1), P(:, 2), P(:, 3), 'b.', 'MarkerSize', 1);
hold on;
title('7自由度机械臂工作空间');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal;
grid on;
daspect([1 1 1]);
view([45 45]);
seven_dof.plot([0 0 0 0 0 0 0]);

