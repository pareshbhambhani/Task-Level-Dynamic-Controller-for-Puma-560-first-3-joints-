%%To generate Yd and Y'd
function [yd yd_dot yd_dot_dot] = traj(w,t)
R = 0.25;
yd = [-0.866*R*cos(w*t)-0.56 ;R*sin(w*t) ; 0.5*R*cos(w*t)-0.08];
yd_dot = [(433*R*w*sin(t*w))/500 ; R*w*cos(t*w); -(R*w*sin(t*w))/2]; %From symbolic derivation script
yd_dot_dot = [ (433*R*w^2*cos(t*w))/500; -R*w^2*sin(t*w); -(R*w^2*cos(t*w))/2]; %From symbolic derivation script
