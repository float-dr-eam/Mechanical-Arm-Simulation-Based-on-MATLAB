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

% 定义起点和终点
% P1=[-0.4,0.0,0.13];
% P2=[0.05,0.3,0.1];
init_pose = transl(-0.5522, 0, 0.1299) * trotx(-90) * troty(-90);
target_pose = transl(0.0, -0.216, 0.466) * trotz(90);

% qi = [0*pi/180, 0*pi/180,0*pi/180, 0*pi/180, 0*pi/180,0*pi/180,0*pi/180];
% qt =[0.9272    1.2036   -0.2800    0.6733   -0.4819   -0.8395   -0.2922];
% 创建时间向量(0到2秒，51个点)
t=linspace(0,2,51);

% 使用我们自己的五次多项式插值函数生成轨迹
%Traj = my_quintic_traj(qi, qt, t);
%Traj = my_quintic_traj(P1, P2, t);
% 方法1: 使用Robotics Toolbox内置的ctraj函数
%T = ctraj(init_pose, target_pose, length(t));

% 方法2: 使用自定义插值函数
T = homogeneous_quintic_traj(init_pose, target_pose, t);

% 显示结果
% figure(1);
% subplot(7,1,1);
% plot(t, Traj(:,1)); title('轴1位置'); grid on;
% subplot(7,1,2);
% plot(t, Traj(:,2)); title('轴2位置'); grid on;
% subplot(7,1,3);
% plot(t, Traj(:,3)); title('轴3位置'); grid on;
% subplot(7,1,4);
% plot(t, Traj(:,4)); title('轴4位置'); grid on;
% subplot(7,1,5);
% plot(t, Traj(:,5)); title('轴5位置'); grid on;
% subplot(7,1,6);
% plot(t, Traj(:,6)); title('轴6位置'); grid on;
% subplot(7,1,7);
% plot(t, Traj(:,7)); title('轴7位置'); grid on;

% figure(1);
% subplot(3,1,1);
% plot(t, Traj(:,1)); title('x轴位置'); grid on;
% subplot(3,1,2);
% plot(t, Traj(:,2)); title('y轴位置'); grid on;
% subplot(3,1,3);
% plot(t, Traj(:,3)); title('z轴位置'); grid on;
%为每个轨迹点创建变换矩阵
% n = size(Traj, 1);
% T = zeros(4,4,n);
% for i = 1:n
%     T(:,:,i) = transl(Traj(i,:))* trotx(180);%
% end

% 使用逆运动学求解关节角度
Qtraj = seven_dof.ikunc(T);


figure(2);
%seven_dof.plot(Qtraj,'trail','b');
seven_dof.plot(Qtraj,'view', [-150,25],'workspace', [-0.5, 0.5, -0.5, 0.5, -0.5, 0.7],'trail','b','movie','trail.gif');

% 定义多维五次多项式轨迹规划函数
function traj = my_quintic_traj(p0, pf, t_samples)
    % 计算轨迹时间
    tf = t_samples(end);
    
    % 获取点的维度
    dim = length(p0);
    num_points = length(t_samples);
    
    % 初始化轨迹矩阵
    traj = zeros(num_points, dim);
    
    % 对每个维度分别计算五次多项式轨迹
    for i = 1:dim
        % 计算系数
        a0 = p0(i);
        a1 = 0;
        a2 = 0;
        a3 = 10*(pf(i)-p0(i))/(tf^3);
        a4 = -15*(pf(i)-p0(i))/(tf^4);
        a5 = 6*(pf(i)-p0(i))/(tf^5);
        
        % 计算每个时间点的位置
        for j = 1:num_points
            t = t_samples(j);
            t2 = t^2;
            t3 = t^3;
            t4 = t^4;
            t5 = t^5;
            traj(j,i) = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        end
    end
end

function T = homogeneous_quintic_traj(T0, Tf, t_samples)
    % 计算轨迹时间
    tf = t_samples(end);
    num_points = length(t_samples);
    
    % 提取初始和终止位置
    p0 = transl(T0);
    pf = transl(Tf);
    % 提取初始和终止旋转矩阵
    R0 = T0(1:3, 1:3);
    Rf = Tf(1:3, 1:3);
    
    % 将旋转矩阵转换为轴角表示
    [theta0,k0] = tr2angvec(R0);
    [thetaf,kf ] = tr2angvec(Rf);
    
    % 确保旋转方向一致性
    if dot(k0, kf) < 0
        kf = -kf;
        thetaf = -thetaf;
    end
    
    % 初始化轨迹矩阵
    T = zeros(4, 4, num_points);
    
    % 五次多项式系数 - 位置
    a0_p = p0;
    a1_p = zeros(3,1);
    a2_p = zeros(3,1);
    a3_p = 10*(pf-p0)/(tf^3);
    a4_p = -15*(pf-p0)/(tf^4);
    a5_p = 6*(pf-p0)/(tf^5);
    %fprintf('  p3 (大小: %s)\n', mat2str(size(a3_p)));
    % 五次多项式系数 - 角度
    a0_r = theta0;
    a1_r = 0;
    a2_r = 0;
    a3_r = 10*(thetaf-theta0)/(tf^3);
    a4_r = -15*(thetaf-theta0)/(tf^4);
    a5_r = 6*(thetaf-theta0)/(tf^5);
    
    % 计算每个时间点的变换矩阵
    for i = 1:num_points
        t = t_samples(i);
        t2 = t^2;
        t3 = t^3;
        t4 = t^4;
        t5 = t^5;
        
        % 计算位置
        p = a0_p + a1_p*t + a2_p*t2 + a3_p*t3 + a4_p*t4 + a5_p*t5;
        %fprintf('  p (大小: %s)\n', mat2str(size(p)));

        % 计算旋转角度
        theta = a0_r + a1_r*t + a2_r*t2 + a3_r*t3 + a4_r*t4 + a5_r*t5;
        
        % 线性插值旋转轴（可以使用球面插值SLERP改进）
        k = k0 + (kf-k0)*t/tf;
        k = k/norm(k);  % 标准化旋转轴
      
        % 打印theta和k的结构信息
        % fprintf('时间点 %d:\n', i);
        % fprintf('  theta = %.6f (大小: %s)\n', theta, mat2str(size(theta)));
        % fprintf('  k = [%.6f, %.6f, %.6f] (大小: %s)\n', k(1), k(2), k(3), mat2str(size(k)));
        % 构建旋转矩阵
        R = angvec2r(theta, k);
        
        % 构建变换矩阵
        T(:,:,i) = [R, p; 0 0 0 1];

    end
end