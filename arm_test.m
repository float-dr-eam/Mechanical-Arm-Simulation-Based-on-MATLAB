
%clc; clear; close all;
syms q1 q2 q3 q4 q5 q6 q7 real

% DH参数
alpha = [0, pi/2, -pi/2, -pi/2, pi/2, -pi/2, pi/2];
a     = [0, 0, 0, 0, 0, 0, 0];
d     = [0.1299, 0, 0.2153, 0, 0.2163, 0, 0.1206];
theta = [q1, pi/2+q2, -pi/2+q3, q4, q5, q6, q7];

% 依次计算每一节的变换
T = sym(eye(4));
for i = 1:7
    T = T * dh_matrix(a(i), alpha(i), d(i), theta(i));
end

% T = simplify(T);

disp('末端执行器的齐次变换矩阵T07为：');
disp(T);

% 你可以继续提取位置、姿态，或者数值代入
% 关节角度数值（单位：弧度）
q_num = [0*pi/180, 0*pi/180, 72*pi/180, -126*pi/180, 0*pi/180,0*pi/180,0*pi/180];

T_num = double(subs(T, [q1 q2 q3 q4 q5 q6 q7], q_num));

disp('末端执行器位姿（数值）：');
disp(T_num);

% 末端位置
position = T_num(1:3,4);
disp('末端位置 (m):');
disp(position');

R = T_num(1:3, 1:3);
[euler_z, euler_y, euler_x] = dcm2angle(R, 'ZYX');
disp('末端姿态 (ZYX欧拉角, 弧度):');
disp([euler_z, euler_y, euler_x]);

function T = dh_matrix(a, alpha, d, theta)
    T =  [cos(theta),            -sin(theta),             0,            a;
         cos(alpha)*sin(theta),  cos(alpha)*cos(theta),   -sin(alpha), -d*sin(alpha);
         sin(alpha)*sin(theta),  sin(alpha)*cos(theta),   cos(alpha),   d*cos(alpha);
         0,                      0,                       0,            1];
         
end