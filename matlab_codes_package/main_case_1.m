clear all; clc; close all;

%%Parameters
time_start = 0;
step = 0.05;
total_time = 20;
w = pi/4; %omega case 1 for Y(0) = [-0.7765 ; 0 ; 0.045]
iteration = 1;
Kd = [36;31;38];
Kp = [23;9;25];
err_x = zeros(1,400);
err_y = zeros(1,400);
err_z = zeros(1,400);
err_x_dot = zeros(1,400);
err_y_dot = zeros(1,400);
err_z_dot = zeros(1,400);

%For implementing variance in Dynamics , per=0 means no variance
per = 5; %Percentage variation needed
R = rand(3,3);
for i = 1:3
    for j = 1:3
        if R(i,j) >= 0.5
            R(i,j) = (1 + (per/100));
        else
            R(i,j) = (1 - (per/100));
        end
    end
end

%Initial values for first run
q = [0.1932  ; -2.5912  ;  0.6370]; %Calculated using the inverse_kinematics script
q_dot = [0;0;0]; %Since Y'(0) = [0;0;0]
Y_init = [-0.7765 ; 0 ; 0.045];
Y_dot_init = [0;0;0];
[Yd, Yd_dot, Yd_dot_dot] = traj(w,0);
e = Yd - Y_init;
e_dot = Yd_dot - Y_dot_init;
u = Yd_dot_dot + Kp.*e + Kd.*e_dot; % u = Yd" + Kd(e') + Kp(e) and initially Yd' and Yd" = 0 and e = Yd
tau = non_linear_fb(u,q,q_dot);

while iteration*step < total_time
    
    %%Dynamics Block%%
    % takes in tau and gives out q and q'
    [T,X]=ode45(@(t,x) dynamics_var(x,tau,R),[time_start step],[q,q_dot]);
    q = X(length(X),1:3)';
    q_dot = X(length(X),4:6)';
    

    %%Forward Kinematics Block%%
    %Takes in q,q' from dynamics block and gives out Y and Y'
    [Y, Y_dot] = forward_kinematics(q,q_dot);

    %%Desired trajectory%%
    %Takes in omega and time and gives out Yd and Yd'
    [Yd, Yd_dot, Yd_dot_dot] = traj(w,(iteration*step));
    
    
    %%Error%%
    %Calulate the error input to the linear controller
    e = Yd - Y;
    e_dot = Yd_dot - Y_dot;
    
    
    %%Linear Controller%%
    % u = Yd" + Kd(e') + Kp(e)
    %Takes in the error and gives out u
    u = Yd_dot_dot + Kd.*e_dot + Kp.*e;
    
    %%Non-Linear feedback%%
    %Takes in u from linear controller and gives out tau
    tau = non_linear_fb(u,q,q_dot);
    
    iteration = iteration + 1;
    err_x(iteration) = e(1);
    err_y(iteration) = e(2);
    err_z(iteration) = e(3);
    err_x_dot(iteration) = e_dot(1);
    err_y_dot(iteration) = e_dot(2);
    err_z_dot(iteration) = e_dot(3);
end
%Plot of error and error_dot
figure(01)
t=0:0.05:19.95;
A=subplot(3,2,1);
plot(t,err_x)
set( get(A,'YLabel'), 'String', 'error x' )
B=subplot(3,2,3)
plot(t,err_y);
set( get(B,'YLabel'), 'String', 'error y' )
C=subplot(3,2,5)
plot(t,err_z)
set( get(C,'YLabel'), 'String', 'error z' )
set( get(C,'XLabel'), 'String', 'time' )
D=subplot(3,2,2)
plot(t,err_x_dot)
set( get(D,'YLabel'), 'String', 'rate error x' )
E=subplot(3,2,4)
plot(t,err_y_dot)
set( get(E,'YLabel'), 'String', 'rate error y' )
F=subplot(3,2,6)
plot(t,err_z_dot)
set( get(F,'YLabel'), 'String', 'rate error z' )
set( get(F,'XLabel'), 'String', 'time' )