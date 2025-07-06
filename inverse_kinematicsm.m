% 七轴机械臂逆运动学求解
% 使用迭代法求解七轴机械臂的逆运动学问题

%% 主函数
function inverse_kinematicsm()
    % 清空工作区和命令窗口
    clc;
    clear;
    
    % 创建七轴机械臂模型
    robot = create_7dof_robot();
    
    % 显示机械臂初始状态
    q_init = zeros(1, 7);  % 初始关节角度（行向量）
    % figure(1);
    % robot.plot(q_init);
    % title('七轴机械臂初始状态');
    
    % 设置目标位姿
    target_pose = transl(0.0, -0.216, 0.466) * trotz(90);
    target_pose
    % 多初值尝试策略
    num_attempts = 3;  % 尝试次数
    best_q = q_init;
    best_error = inf;
    success = false;
    
    for attempt = 1:num_attempts
        % 生成随机初始值（在关节限位范围内）
        if attempt == 1
            current_q_init = q_init;  % 第一次使用零位姿态
        else
            current_q_init = generate_random_config(robot);
        end
        
        % 求解逆运动学
        [q_sol, current_success, final_error] = inverse_kinematics_solve(robot, target_pose, current_q_init);
        
        % 更新最佳结果
        if current_success || (final_error < best_error)
            best_q = q_sol;
            best_error = final_error;
            success = current_success;
            if current_success
                break;  % 如果找到满足要求的解，直接退出
            end
        end
        
        disp(['尝试 ', num2str(attempt), ' 完成，当前最佳误差: ', num2str(best_error)]);
    end
    
    % 显示最终结果
    if success
        disp('逆运动学求解成功！');
        disp('关节角度结果（弧度）:');
        disp(best_q);
        
        % 验证结果
        T_result = robot.fkine(best_q);
        disp('目标位姿:');
        disp(target_pose);
        disp('求解结果位姿:');
        disp(T_result);


        % 提取位置误差
        if isobject(T_result) || isstruct(T_result)
            % 如果是对象或结构体，使用transl函数可能返回不同格式
            p_current = T_result.t;  % 直接获取平移向量
        else
            % 如果是矩阵
            p_current = T_result(1:3, 4);
        end
        
        if isobject(target_pose) || isstruct(target_pose)
            p_target = target_pose.t;
        else
            p_target = target_pose(1:3, 4);
        end
        
        % 确保p_current和p_target是3x1列向量
        p_current = p_current(:);
        p_target = p_target(:);
        
        % 计算位置误差
        dp = p_target - p_current;  % 3×1列向量
        
        % 计算误差
        pos_error = norm(dp);
        disp(['位置误差: ', num2str(pos_error), ' 米']);
        
        % % 显示最终状态
        % figure(2);
        % robot.plot(best_q);
        % %robot.plot(best_q, 'PreservePlot', false);
        % title('七轴机械臂最终状态');
        
        % 动画演示
        figure(1);
        steps = 50;
        q_traj = zeros(steps, 7);
        for i = 1:steps
            q_traj(i, :) = q_init + (best_q - q_init) * (i-1)/(steps-1);
        end
        
        for i = 1:steps
            robot.plot(q_traj(i, :));
            title('七轴机械臂运动轨迹');
            drawnow;
            pause(0.05);
        end
    else
        disp(['逆运动学求解失败！最佳误差: ', num2str(best_error)]);
    end
end

%% 创建七轴机械臂模型
function robot = create_7dof_robot()
    % 使用SerialLink创建机械臂模型
    % 注意：这里使用的DH参数需要根据实际机械臂进行调整
    
    % 创建连杆
    L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0,'modified');
    L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2,'modified');
    L(3) = Link('revolute', 'd', 0.2153, 'a', 0, 'alpha', pi/2, 'offset', pi/2,'modified');
    L(4) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
    L(5) = Link('revolute', 'd', 0.2163, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');
    L(6) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 0,'modified');
    L(7) = Link('revolute', 'd', 0.1206, 'a', 0, 'alpha', -pi/2, 'offset', 0,'modified');

    % 设置关节限位
    for i = 1:7
        L(i).qlim = [-pi, pi];
    end
    
    % 创建机械臂模型
    robot = SerialLink(L, 'name', '7-DOF Robot');
end

%% 逆运动学求解函数
function [q_sol, success, final_error] = inverse_kinematics_solve(robot, T_target, q_init)
    % 参数设置
    max_iter = 5000;      % 最大迭代次数
    epsilon = 0.00001;       % 收敛阈值
    lambda_max = 0.1;     % 最大阻尼因子
    lambda_min = 0.0001;   % 最小阻尼因子
    step_size = 0.5;      % 步长因子
    
    % 初始化
    q = q_init;
    success = false;
    error_history = zeros(1, max_iter);  % 预分配误差历史数组
    stagnation_count = 0;
    lambda = lambda_max;  % 初始阻尼因子
    
    % 迭代求解
    for i = 1:max_iter
       
        T_current = robot.fkine(q); % 计算当前位姿
        delta_x = compute_pose_error(T_current, T_target); % 计算位姿误差
        error_norm = norm(delta_x);
        error_history(i) = error_norm; 
        
        % 显示当前迭代信息
        if mod(i, 100) == 0 || i == 1
            disp(['迭代: ', num2str(i), ', 误差: ', num2str(error_norm)]);
        end
        
        % 检查是否收敛
        if error_norm < epsilon
            success = true;
            disp(['成功收敛! 迭代次数: ', num2str(i), ', 最终误差: ', num2str(error_norm)]);
            error_history = error_history(1:i);  % 截断未使用的部分
            break;
        end
        
        % 计算雅可比矩阵
        J = robot.jacob0(q);
        
        % 自适应阻尼因子策略
        if i > 1
            if error_norm < error_history(i-1)% 误差减小，减小阻尼因子
                lambda = max(lambda_min, lambda * 0.7);
            else
                lambda = min(lambda_max, lambda * 1.3); % 误差增大，增加阻尼因子
            end
        end
        
        
        q_mid = zeros(1, 7);
        cost_grad = (q - q_mid)';% 处理冗余度 - 使用零空间投影法避免关节限位
        
        % 1、计算阻尼最小二乘解
        J_pinv = J' *(inv(J*J' + lambda^2*eye(6))) ;% 右伪逆 6,7 *7,6=6,6
        %                  7,6 * 6,7 *7,6=7,6

        % 2、 计算阻尼伪逆 (更稳定的计算方式)
        % H = J'*J + lambda^2*eye(7);
        % J_pinv = H \ J';

        % 计算零空间投影矩阵
        N = eye(7) - J_pinv * J;% 投影矩阵 (I−J-*J) 
        %      7,6 * 6,7 = 7,7
        
        % 更新关节角度（主任务 + 零空间优化）
        alpha = 0.00;  % 零空间任务权重
        dq = J_pinv * delta_x - alpha * N * cost_grad;
        
        % 检查是否陷入局部最小值
        if i > 10
            recent_errors = error_history(max(1, i-9):i);
            if std(recent_errors) < epsilon  % 如果最近的误差变化很小
                stagnation_count = stagnation_count + 1;
                if stagnation_count > 5      % 连续多次陷入停滞
                    % 添加随机扰动
                    random_dq = (rand(7,1) - 0.5) * 0.1;  % 小幅随机扰动
                    dq = dq + N * random_dq;              % 在零空间添加扰动
                    % disp('检测到局部最小值，添加随机扰动...');
                    stagnation_count = 0;  % 重置停滞计数器
                end
            else
                stagnation_count = 0;
            end
        end
        
        % 步长控制
        if norm(dq) > 0.5
            dq = 0.5 * dq / norm(dq);
        end
        
        % 更新关节角度
        q = q + step_size * dq';
        
        %关节限位处理
        for j = 1:7
            if q(j) < robot.links(j).qlim(1)
                q(j) = robot.links(j).qlim(1);
            elseif q(j) > robot.links(j).qlim(2)
                q(j) = robot.links(j).qlim(2);
            end
        end
        
        % 检查是否发散
        if i > 10 && error_history(i) > 2 * error_history(i-10)
            disp('误差增大，算法可能发散，尝试减小步长...');
            step_size = step_size * 0.5;
            if step_size < 0.01
                disp('步长过小，算法终止');
                error_history = error_history(1:i);  % 截断未使用的部分
                break;
            end
        end
    end
    
    % 记录最终误差
    final_error = error_norm;
    
    if ~success
        disp(['未收敛! 最大迭代次数: ', num2str(max_iter), ', 最终误差: ', num2str(error_norm)]);
        
        % 绘制误差变化曲线
        figure;
        plot(1:length(error_history), error_history);
        title('逆运动学求解误差变化');
        xlabel('迭代次数');
        ylabel('误差范数');
        grid on;
    end
    
    q_sol = q;
end

%% 计算位姿误差
function delta_x = compute_pose_error(T_current, T_target)
    % 提取位置误差
    if isobject(T_current) || isstruct(T_current)
        % 如果是对象或结构体，使用transl函数可能返回不同格式
        p_current = T_current.t;  % 直接获取平移向量
    else
        % 如果是矩阵
        p_current = T_current(1:3, 4);
    end
    
    if isobject(T_target) || isstruct(T_target)
        p_target = T_target.t;
    else
        p_target = T_target(1:3, 4);
    end
    
    % 确保p_current和p_target是3x1列向量
    p_current = p_current(:);
    p_target = p_target(:);
    
    % 计算位置误差
    dp = p_target - p_current;  % 3×1列向量
    
    % 提取旋转矩阵
    if isobject(T_current) || isstruct(T_current)
        % 如果T_current是对象或结构体，使用.R属性
        R_current = T_current.R;
    else
        % 否则从矩阵中提取
        R_current = T_current(1:3, 1:3);
    end
    
    if isobject(T_target) || isstruct(T_target)
        R_target = T_target.R;
    else
        R_target = T_target(1:3, 1:3);
    end
    
    % 计算旋转误差
    dR = R_target * R_current';
    %dR = R_current'* R_target  ;
    
    % 将旋转误差转换为角速度形式
    [w_angle, w_axis] = tr2angvec(dR);
    w_axis = w_axis(:);  % 确保是列向量
    dw = w_axis * w_angle;  % 3×1列向量
    
    % 组合位姿误差
    delta_x = [dp; dw];  % 6×1列向量
    
end

%% 辅助函数：计算伪逆
function J_pinv = compute_pseudoinverse(J)
    % 使用SVD计算伪逆，提高数值稳定性
    [U, S, V] = svd(J);
    
    % 处理奇异值
    tol = max(size(J)) * eps(max(diag(S)));
    r = sum(diag(S) > tol);
    
    if r < size(S, 1)
        % 如果存在很小的奇异值，进行处理
        S_inv = zeros(size(S'));
        for i = 1:r
            S_inv(i, i) = 1/S(i, i);
        end
        J_pinv = V * S_inv * U';
    else
        % 标准伪逆
        J_pinv = V * pinv(S) * U';
    end
end

%% 生成随机构型
function q = generate_random_config(robot)
    q = zeros(1, 7);
    for i = 1:7
        limits = robot.links(i).qlim;
        q(i) = limits(1) + (limits(2) - limits(1)) * rand();
    end
end