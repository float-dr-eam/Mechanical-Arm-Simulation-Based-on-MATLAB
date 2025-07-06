% 圆弧轨迹规划主函数
function circle_tra()
    % 1. 定义机器人模型（使用之前的7自由度机械臂）
    L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0,'modified');
    L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2,'modified');
    L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', pi/2, 'offset', pi/2,'modified');
    L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
    L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
    L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
    L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
    
    % 创建串联机械臂对象
    robot = SerialLink(L, 'name', '7-dof');
    robot.base = transl(0, 0, 0.1299);
    
    % 2. 定义圆弧的三个点
    p_start = [-0.5522, 0, 0.1299];      % 圆弧轨迹起始点
    p_mid   = [-0.2, -0.20, 0.20];    % 圆弧轨迹中间点
    p_final = [0.0, -0.216, 0.466];   % 圆弧轨迹终止点
    
    % 定义起始姿态和终止姿态（RPY角表示，单位：弧度）
    start_pose = transl(p_start) * trotx(-90) * troty(-90);   % 起始姿态
    final_pose = transl(p_final) * trotz(90); % 终止姿态
    % 3. 计算圆弧轨迹并生成变换矩阵
    num_points = 50;                    % 轨迹点数
    [T, traj_points, center, radius] = calculate_circle_trajectory( p_mid, start_pose, final_pose, num_points);
    
    % 4. 使用逆运动学求解关节角度
    Qtraj = robot.ikunc(T);
    
    % 5. 可视化结果
    figure (1);
    
    % 绘制圆弧轨迹
    %subplot(1,2,1);
    plot3(traj_points(:,1), traj_points(:,2), traj_points(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3([p_start(1), p_mid(1), p_final(1)], [p_start(2), p_mid(2), p_final(2)], [p_start(3), p_mid(3), p_final(3)], 'ro', 'MarkerSize', 8);
    plot3(center(1), center(2), center(3), 'gx', 'MarkerSize', 10);
    
    % 绘制起始和终止姿态
    plotFrame(start_pose, 'r', 0.05);
    plotFrame(final_pose, 'g', 0.05);
    
    grid on;
    xlabel('X轴(m)');
    ylabel('Y轴(m)');
    zlabel('Z轴(m)');
    title('圆弧轨迹规划');
    legend('轨迹', '关键点', '圆心', '起始姿态', '终止姿态');
    
    % 绘制机器人动画
    %subplot(1,2,2);
    figure (2);
    robot.plot(Qtraj,'view', [-150,25],'workspace', [-0.5, 0.5, -0.5, 0.5, -0.5, 0.7], 'fps', 10,'movie','trail22.gif','trail', {'r', 'LineWidth', 2});
    
    % 6. 保存轨迹数据
    save('circle_traj.mat', 'traj_points', 'Qtraj', 'center', 'radius', 'T');
end

% 计算圆弧轨迹和对应的变换矩阵，包括姿态插值
function [T, traj_points, center, radius] = calculate_circle_trajectory( p_mid, start_pose, final_pose, num_points)
    p_start = start_pose(1:3,4)';
    p_final = final_pose(1:3,4)';
    % 1. 计算三点构成的三角形各边长
    a = norm(p_final - p_mid);          % 内接三角形边长a
    b = norm(p_final - p_start);        % 内接三角形边长b
    c = norm(p_mid - p_start);          % 内接三角形边长c
    
    % 2. 计算圆的半径
    s = (a + b + c) / 2;               % 三角形半周长
    radius = a*b*c / (4 * sqrt(s*(s-a)*(s-b)*(s-c))); % 外接圆半径
    
    % 3. 计算圆心
    % 通过权重计算圆心坐标
    b1 = a*a * (b*b + c*c - a*a);
    b2 = b*b * (a*a + c*c - b*b);
    b3 = c*c * (a*a + b*b - c*c);
    P1 = [p_start'  p_mid'  p_final'];
    P2 = [b1; b2; b3];
    P3 = P1 * P2;
    center = P3 ./ (b1 + b2 + b3);
    center = center';  % 转置为行向量
    
    % 4. 计算平面法向量和坐标系
    % 计算从圆心到起点的向量
    vector_start_big = p_start - center;                          
    vector_start = vector_start_big / norm(vector_start_big);  % 单位化
    
    % 计算从圆心到终点的向量
    vector_final = (p_final - center) / norm(p_final - center);  
    
    % 计算旋转轴（平面法向量）
    rotation_axis = cross(vector_start, vector_final);   
    rotation_axis = rotation_axis / norm(rotation_axis); % 单位化
    
    % 5. 计算夹角
    theta = acos(dot(vector_start, vector_final));  % 弧度制的圆弧角度
    
    % 检查旋转方向，确保圆弧经过中间点
    % 计算中间点的预期角度
    vector_mid = (p_mid - center) / norm(p_mid - center);
    theta_mid = acos(dot(vector_start, vector_mid));
    
    % 检查旋转方向
    test_point = rotation_matrix(rotation_axis, theta_mid) * vector_start_big';
    test_point = center + test_point';
    
    % 如果计算出的点与中间点距离过大，可能需要反向旋转
    if norm(test_point - p_mid) > 0.1 * radius
        theta = 2*pi - theta;  % 使用补角
    end
    
    % 6. 生成轨迹点
    theta_per = theta / (num_points - 1);  % 每段角度增量
    traj_points = zeros(num_points, 3);    % 轨迹点坐标
    T = zeros(4, 4, num_points);           % 变换矩阵
    
    % 提取起始和终止姿态的旋转矩阵
    R_start = start_pose(1:3, 1:3);
    R_end = final_pose(1:3, 1:3);
    
    % 转换为四元数进行插值
    q_start = rotm2quat(R_start);
    q_end = rotm2quat(R_end);
    
    % 确保取最短路径
    if dot(q_start, q_end) < 0
        q_end = -q_end;
    end
    
    for i = 1:num_points
        % 当前角度
        theta_current = (i-1) * theta_per;
        
        % 计算旋转矩阵
        R_current = rotation_matrix(rotation_axis, theta_current);
        
        % 计算当前点坐标
        vector_current = R_current * vector_start_big';
        p_current = center + vector_current';
        traj_points(i, :) = p_current;
        
        % 姿态插值参数（0到1）
        s = (i-1) / (num_points-1);
        
        % 使用SLERP进行四元数插值
        q_current = slerp(q_start, q_end, s);
        
        % 转换为旋转矩阵
        R = quat2rotm(q_current);
        
        % 构建变换矩阵
        T(1:3, 1:3, i) = R;
        T(1:3, 4, i) = p_current';
        T(4, :, i) = [0 0 0 1];
    end
end

% 四元数球面线性插值(SLERP)函数
function q = slerp(q1, q2, t)
    % 确保取最短路径
    cosHalfTheta = dot(q1, q2);
    if cosHalfTheta < 0
        q1 = -q1;
        cosHalfTheta = -cosHalfTheta;
    end
    
    % 如果q1和q2非常接近，执行线性插值
    if cosHalfTheta > 0.9999
        q = q1 + t*(q2 - q1);
        q = q / norm(q);
        return;
    end
    
    % 否则执行球面线性插值
    halfTheta = acos(cosHalfTheta);
    sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    
    % 如果theta = 180度，结果不唯一
    if abs(sinHalfTheta) < 0.001
        q = q1*0.5 + q2*0.5;
        q = q / norm(q);
        return;
    end
    
    ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
    ratioB = sin(t * halfTheta) / sinHalfTheta;
    
    q = q1*ratioA + q2*ratioB;
    q = q / norm(q);
end

% 计算旋转矩阵（绕任意轴旋转）
function R = rotation_matrix(axis, theta)
    % 确保轴是单位向量
    axis = axis / norm(axis);
    
    % 使用四元数表示旋转
    a = cos(theta/2);
    b = -axis(1) * sin(theta/2);
    c = -axis(2) * sin(theta/2);
    d = -axis(3) * sin(theta/2);
    
    % 构建旋转矩阵
    R = [
        a^2+b^2-c^2-d^2,     2*(b*c+a*d),     2*(b*d-a*c);
        2*(b*c-a*d),     a^2-b^2+c^2-d^2,     2*(c*d+a*b);
        2*(b*d+a*c),     2*(c*d-a*b),     a^2-b^2-c^2+d^2
    ];
end

% RPY角转换为旋转矩阵
function R = rpy2r(roll, pitch, yaw)
    % 绕X轴旋转
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    % 绕Y轴旋转
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    % 绕Z轴旋转
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    
    % 组合旋转
    R = Rz * Ry * Rx;
end

% 绘制坐标系函数
function plotFrame(T, color, scale)
    % 提取原点和轴向量
    o = T(1:3, 4)';
    x = o + scale * T(1:3, 1)';
    y = o + scale * T(1:3, 2)';
    z = o + scale * T(1:3, 3)';
    
    % 绘制原点和三个坐标轴
    plot3(o(1), o(2), o(3), [color 'o'], 'MarkerSize', 6, 'MarkerFaceColor', color);
    line([o(1) x(1)], [o(2) x(2)], [o(3) x(3)], 'Color', 'r', 'LineWidth', 2); % X轴-红色
    line([o(1) y(1)], [o(2) y(2)], [o(3) y(3)], 'Color', 'g', 'LineWidth', 2); % Y轴-绿色
    line([o(1) z(1)], [o(2) z(2)], [o(3) z(3)], 'Color', 'b', 'LineWidth', 2); % Z轴-蓝色
end