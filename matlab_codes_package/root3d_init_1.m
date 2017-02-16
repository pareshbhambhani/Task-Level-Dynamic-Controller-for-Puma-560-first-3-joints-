%%Forward kinematics model to be used to calculate q(0) and q'(0) via
%%inverse kinematics when Y(0) =  [-0.7765 ; 0 ; 0.045]

function F = root3d_init_1(q)
%Robot parameters
a = [0 431.8/1000 -20.32/1000 0]; %ai in m
d = [0 149.09/1000 0 433.07/1000]; %di in m
Y = [-0.7765 ; 0 ; 0.045];

F(1) = a(3)*cos(q(1))*cos(q(2)+q(3)) + d(4)*cos(q(1))*sin(q(2)+q(3)) + a(2)*cos(q(1))*cos(q(2)) - d(2)*sin(q(1)) - Y(1);
F(2) = a(3)*sin(q(1))*cos(q(2)+q(3)) + d(4)*sin(q(1))*sin(q(2)+q(3)) + a(2)*sin(q(1))*cos(q(2)) + d(2)*cos(q(1)) - Y(2);
F(3) = - a(3)*sin(q(2)+q(3)) + d(4)*cos(q(2)+q(3)) - a(2)*sin(q(2)) - Y(3);