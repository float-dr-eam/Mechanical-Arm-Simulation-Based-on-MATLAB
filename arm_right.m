% 创建7自由度机械臂的DH参数模型
% 注意：使用Robotics Toolbox for MATLAB

% 定义各连杆参数（根据提供的DH参数表）
L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0,'modified');
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2,'modified');
L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', pi/2, 'offset', pi/2,'modified');
L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');

% 创建串联机械臂对象
seven_dof = SerialLink(L, 'name', '7-dof');

% 设置机械臂基坐标位置（根据需要调整）
seven_dof.base = transl(0, 0, 0.1299);

% 显示机械臂结构和DH参数
% disp('机械臂DH参数表：');
% seven_dof.display();

% 定义关节角度


% 绘制机械臂（移除了不支持的tilecolor选项）
seven_dof.teach();

%q = [0*pi/180, 0*pi/180,0*pi/180, 0*pi/180, 0*pi/180,0*pi/180,0*pi/180];
%q =[0.9272    1.2036   -0.2800    0.6733   -0.4819   -0.8395   -0.2922];
%q = [ -0.068468      1.552635    0.086558  1.571277   -0.017999  -1.569684   -0.018108  ];     
% figure('Name', '七自由度机械臂可视化');
%seven_dof.plot(q,'view', [-150,25]);% ...           % 设置视角 (方位角, 仰角)
    % 'workspace', [-1.5, 1.5, -1.5, 1.5, -1.5, 2], ... % 工作空间范围
    % 'tilesize', 0.5, ...          % 网格大小
    % 'jointlen', 3, ...            % 关节长度
    % 'linkcolor', 'r', ...         % 连杆颜色
    % 'noshadow', ...               % 不显示阴影
    % 'noname', ...                 % 不显示关节名称
    % 'noshading', ...              % 不使用阴影效果
    % 'basecolor', 'b' ...          % 基座颜色 (蓝色)
  %  );            

% 添加标题和坐标轴标签
% title('七自由度机械臂姿态');
% xlabel('X轴');
% ylabel('Y轴');
% zlabel('Z轴');
% grid on;

% figure('Name','1');
% seven_dof.plot(q);
% T= seven_dof. fkine(q);
% T
% J = seven_dof.jacob0(q);
% J

% M=seven_dof.ikunc(T);
% M
% figure('Name','2');
% seven_dof.plot(M); 
% T0= seven_dof. fkine(M);
% T0
% 
% M1=seven_dof.ikine(T);
% M1
% figure('Name','3');
% seven_dof.plot(M1); 
% T1= seven_dof. fkine(M1);
% T1

% M1 = [0 0 0 0 0 0 0];
% seven_dof.plot(M1); 
% 
% pause;
 
% 定义起点和终点
% P1=[-0.5522,0,0.1299]; P2=[0.7,0.5,0.5];
% 
% % 创建时间向量(0到2秒，51个点)
% t=linspace(0,2,51);
% 
% % 使用五次多项式插值生成轨迹
% Traj = mtraj(@tpoly,P1,P2,t);
% 
% % 为每个轨迹点创建变换矩阵
% n = size(Traj, 1);
% T = zeros(4,4,n);
% for i = 1:n
%     T(:,:,i) = transl(Traj(i,:));% * trotx(180);
% end
% 
% % 使用逆运动学求解关节角度
% Qtraj = seven_dof.ikunc(T);
% 
% % 绘制机器人运动轨迹
% % seven_dof.plot(Qtraj);
% seven_dof.plot(Qtraj,'trail','b');
% % seven_dof.plot(Qtraj,'movie','trail.gif');