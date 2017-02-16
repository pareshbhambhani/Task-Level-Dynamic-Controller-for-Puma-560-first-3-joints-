%%Inverse Kinematics for q(0) and q'(0)

%function [q,q_dot] = inverse_kinematics(Yd,Yd_dot)

%Robot parameters
a = [0 431.8/1000 -20.32/1000 0]; %ai in m
d = [0 149.09/1000 0 433.07/1000]; %di in m

%Initial Y and Y'
Y_zero_1 = [-0.7765 ; 0 ; 0.045];
Y_zero_2 = [-0.5 ; -0.1 ; 0];
Y_dot_zero_1 = [0;0;0];
Y_dot_zero_2 = [0;0;0];

%options = optimoptions(@fsolve,'Display','iter','Jacobian','on');
kin_mod_1 = @root3d_init_1;
q0_1 = [0,0,0];
q_init_1 = fsolve(kin_mod_1,q0_1)
q0_2 = [pi/4,-pi/4,pi/4];
kin_mod_2 = @root3d_init_2;
q_init_2 = fsolve(kin_mod_2,q0_2)

q_dot_init_1 = [0;0;0]; %Since Y'_1(0) = [0;0;0]
q_dot_init_2 = [0;0;0]; %Since Y'_2(0) = [0;0;0]

