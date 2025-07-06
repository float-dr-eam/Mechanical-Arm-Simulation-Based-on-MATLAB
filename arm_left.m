% 创建7自由度机械臂的DH参数模型
% 注意：使用Robotics Toolbox for MATLAB

% 定义各连杆参数（根据提供的DH参数表）d=0.1299
L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0,'modified');
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2,'modified');
L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2,'modified');
L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');

% 创建串联机械臂对象
seven_dof = SerialLink(L, 'name', '7-dof');

% 设置机械臂基坐标位置（根据需要调整）
seven_dof.base = transl(0, 0, 0.1299);

% 显示机械臂结构和DH参数
disp('机械臂DH参数表：');
seven_dof.display();

% 打开机械臂示教界面
seven_dof.teach();