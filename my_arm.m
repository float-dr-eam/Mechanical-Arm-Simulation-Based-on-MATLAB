syms q1 q2 q3 q4 q5 q6 q7; % 定义关节变量为符号变量

% 关节1 (α=0, a=0, d=0.1299, θ=q1)

T01 = [cos(q1) -sin(q1) 0 0;
       sin(q1) cos(q1) 0 0;
       0 0 1 0.1299;
       0 0 0 1];

% 关节2 (α=π/2, a=0, d=0, θ=π/2+q2)
T12 = [-sin(q2) -cos(q2) 0 0;
       0 0 -1 0;
       cos(q2) -sin(q2) 0 0;
       0 0 0 1];

% 关节3 (α=-π/2, a=0, d=0.2153, θ=-π/2+q3)
T23 = [sin(q3) cos(q3) 0 0;
       0 0 1 0.2153;
       cos(q3) -sin(q3) 0 0;
       0 0 0 1];

% 关节4 (α=π/2, a=0, d=0, θ=q4)
T34 = [cos(q4) -sin(q4) 0 0;
       0 0 -1 0;
       sin(q4) cos(q4) 0 0;
       0 0 0 1];

% 关节5 (α=-π/2, a=0, d=0.2163, θ=q5)
T45 = [cos(q5) -sin(q5) 0 0;
       0 0 1 0.2163;
       -sin(q5) -cos(q5) 0 0;
       0 0 0 1];

% 关节6 (α=π/2, a=0, d=0, θ=q6)
T56 = [cos(q6) -sin(q6) 0 0;
       0 0 -1 0;
       sin(q6) cos(q6) 0 0;
       0 0 0 1];

% 关节7 (α=-π/2, a=0, d=0.1206, θ=q7)
T67 = [cos(q7) -sin(q7) 0 0;
       0 0 1 0.1206;
       -sin(q7) -cos(q7) 0 0;
       0 0 0 1];

% 总变换矩阵
T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;

% 具体关节角度 
q_values = [0*pi/180, 0*pi/180, 72*pi/180, -126*pi/180, 0*pi/180,0*pi/180,0*pi/180];

% 计算数值结果
T07_numeric = double(subs(T07, [q1, q2, q3, q4, q5, q6, q7], q_values));

% 显示结果
disp('末端执行器位姿矩阵:');
disp(T07_numeric);

% 提取位置 (前3行第4列)
position = T07_numeric(1:3, 4);
disp('末端位置 (m):');
disp(position');

% 提取旋转矩阵
R = T07_numeric(1:3, 1:3);

% 计算ZYX欧拉角
[euler_z, euler_y, euler_x] = dcm2angle(R, 'ZYX');
disp('末端姿态1 (ZYX欧拉角, 弧度):');
disp([euler_z, euler_y, euler_x]);


Euler=rotm2eul(R)

% 计算ZYX欧拉角（手动实现）
% 偏航角 (Yaw, ψ) - 绕Z轴旋转
yaw = atan2(R(2,1), R(1,1));

% 俯仰角 (Pitch, θ) - 绕Y轴旋转
pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));

% 横滚角 (Roll, φ) - 绕X轴旋转
roll = atan2(R(3,2), R(3,3));

% 转换为弧度输出
euler_angles_rad = [yaw, pitch, roll];
disp('末端姿态2 (ZYX欧拉角, 弧度):');
disp(euler_angles_rad);

% 可选：转换为角度显示
euler_angles_deg = euler_angles_rad * 180/pi;
disp('末端姿态 (ZYX欧拉角, 度):');
disp(euler_angles_deg);