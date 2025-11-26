clear; clc;
syms L1 L2 L3 L4 th1 th2 th3 th4 th5 real 

% DH parameters (standard)
% i | alpha | a   | d       | theta
% 1 |  pi/2 | 0   | L1      | 0
% 2 | 0     | L2  | d2      | 0
% 3 | 0     | L3  | d3      | 0
% 4 | pi/2  | 0   | 0       | 0
% 5 | pi/2  | 0   | 0       | 0

L1 = 3; L2 = 3; L3 = 2;
d2 = 0.8; d3 = 0.8;


T = @(theta, d, a, alpha) [ ...
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),             cos(alpha),             d;
    0, 0, 0, 1];


T01 = T(th1, L1, 0, sym(pi)/2)
T12 = T(th2, d2, L2, 0)
T23 = T(th3, d3, L3, 0)
T34 = T(th4, 0, 0, sym(pi)/2)
T45 = T(th5, 0, 0, sym(pi)/2)


T = simplify(T01 * T12 * T23 * T34 * T45)

