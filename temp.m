% 定义符号变量
syms q2 q3

% 关节2参数
a2 = 0;
alpha2 = -pi/2;
d2 = 0;
theta2 = -pi/2 + q2;

% 关节3参数
a3 = 0;
alpha3 = pi/2;
d3 = 0.2153;
theta3 = pi/2 + q3;

% 计算关节2的DH变换矩阵
T2 = dh_matrix(a2, alpha2, d2, theta2);
T2 = simplify(T2); % 化简矩阵

% 计算关节3的DH变换矩阵
T3 = dh_matrix(a3, alpha3, d3, theta3);
T3 = simplify(T3); % 化简矩阵

% 显示结果
disp('关节2的DH变换矩阵:');
disp(T2);
disp('关节3的DH变换矩阵:');
disp(T3);

function T = dh_matrix(a, alpha, d, theta)
    T =  [cos(theta),            -sin(theta),             0,            a;
         cos(alpha)*sin(theta),  cos(alpha)*cos(theta),   -sin(alpha), -d*sin(alpha);
         sin(alpha)*sin(theta),  sin(alpha)*cos(theta),   cos(alpha),   d*cos(alpha);
         0,                      0,                       0,            1];
end    