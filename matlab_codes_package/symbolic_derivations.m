%% Symbolic derivations
%% This script is to calculate the Jacobian and Yd symbolically to use in other scripts to avoid recalculation for each iteration

%Jacobian
% Robot Params
a = [0 431.8/1000 -20.32/1000 0]; %ai in m
d = [0 149.09/1000 0 433.07/1000]; %di in m

syms q1 q2 q3

q_sym = [q1; q2; q3];

h1 = a(3)*cos(q1)*cos(q2+q3) + d(4)*cos(q1)*sin(q2+q3) + a(2)*cos(q1)*cos(q2) - d(2)*sin(q1);
h2 = a(3)*sin(q1)*cos(q2+q3) + d(4)*sin(q1)*sin(q2+q3) + a(2)*sin(q1)*cos(q2) + d(2)*cos(q1);
h3 = - a(3)*sin(q2+q3) + d(4)*cos(q2+q3) - a(2)*sin(q2);
h = [h1; h2; h3];

Y = h % Y = h(q)

J = jacobian(h,q_sym)

%% Yd' and Yd"

syms t w R
yd = [-0.866*R*cos(w*t)-0.56 ;R*sin(w*t) ; 0.5*R*cos(w*t)-0.08];
yd_dot = diff(yd,t)
yd_dot_dot = diff(yd,t,2)