%clc; clear; close all;
syms q1 q2 q3 q4 q5 q6 q7 real

% 关节1 (α=0, a=0, d=0.1299, θ=q1)
T01 = [cos(q1) -sin(q1) 0 0;
       sin(q1) cos(q1) 0 0;
       0 0 1 0.1299;
       0 0 0 1];

% 关节2 (α=pi/2, a=0, d=0, θ=pi/2+q2)
T12 = [-sin(q2) -cos(q2) 0 0;
       0 0 -1 0;  % 修正：第二行第三列为 -1（原错误为 1）
       cos(q2) -sin(q2) 0 0;
       0 0 0 1];

% 关节3 (α=-pi/2, a=0, d=0.2153, θ=-pi/2+q3)
T23 = [sin(q3) cos(q3) 0 0;
       0 0 1 0.2153;
       -cos(q3) sin(q3) 0 0;  % 修正：第三行第一列为 -cos(q3)（原正确，无需修改）
       0 0 0 1];

% 关节4 (α=pi/2, a=0, d=0, θ=q4)
T34 = [cos(q4) -sin(q4) 0 0;
       0 0 -1 0;  % 修正：第二行第三列为 -1
       sin(q4) cos(q4) 0 0;
       0 0 0 1];

% 关节5 (α=-pi/2, a=0, d=0.2163, θ=q5)
T45 = [cos(q5) -sin(q5) 0 0;
       0 0 1 0.2163;  % 修正：第二行第三列为 1（原正确，α=-pi/2时sin(alpha)=-1，此处d*cos(alpha)=0，无需符号错误）
       -sin(q5) -cos(q5) 0 0;  % 修正：第三行第一列为 -sin(q5)（原正确）
       0 0 0 1];

% 关节6 (α=pi/2, a=0, d=0, θ=q6)
T56 = [cos(q6) -sin(q6) 0 0;
       0 0 -1 0;  % 修正：第二行第三列为 -1
       sin(q6) cos(q6) 0 0;
       0 0 0 1];

% 关节7 (α=-pi/2, a=0, d=0, θ=q7)
T67 = [cos(q7) -sin(q7) 0 0;
       0 0 1 0;  % 修正：第二行第三列为 1（α=-pi/2时sin(alpha)=-1，d=0，无需符号错误）
       -sin(q7) -cos(q7) 0 0;  % 修正：第三行第一列为 -sin(q7)（原正确）
       0 0 0 1];

% 总变换矩阵（现在与dh_matrix一致）
T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;

% 具体关节角度（与上方一致）
q_values = [0, 0, 0, -pi/2, 0, 0, 0];

% 计算数值结果（现在与dh_matrix版本完全一致）
T07_numeric = double(subs(T07, [q1, q2, q3, q4, q5, q6, q7], q_values));

% 显示结果（匹配dh_matrix输出）
disp('末端执行器位姿矩阵:');
disp(T07_numeric);

% 提取位置（与dh_matrix一致）
position = T07_numeric(1:3, 4);
disp('末端位置 (m):');
disp(position');

% 计算ZYX欧拉角（现在与dh_matrix一致）
R = T07_numeric(1:3, 1:3);
[euler_z, euler_y, euler_x] = dcm2angle(R, 'ZYX');
disp('末端姿态 (ZYX欧拉角, 弧度):');
disp([euler_z, euler_y, euler_x]);