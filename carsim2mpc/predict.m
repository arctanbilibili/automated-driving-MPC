function [pre_x,pre_y,pre_phi,pre_vx,pre_vy,pre_phi_dot] = predict(x,y,phi,vx,vy,phi_dot)
% 预测函数（模拟司机）
% Detailed explanation goes here

ts=0.1;
pre_x=x+vx*ts*cos(phi)-vy*ts*sin(phi);
pre_y=y+vy*ts*cos(phi)+vx*ts*sin(phi);
pre_phi=phi+phi_dot*ts;
pre_vx=vx;
pre_vy=vy;
pre_phi_dot=phi_dot;

end