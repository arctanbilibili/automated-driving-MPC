% [A,B] = getAB(vx,a,b,m,cf,cr,Iz,Ts);

A=ones(4);
B=ones(4,1);
err=0.5*ones(4,1);


C = eye(4);
Np = 15;
Nc = 5;
Q = [100 5 50 10];
Q = diag([Q,Q,Q,Q,Q,Q,Q,Q,Q,Q,Q,Q,Q,Q,Q]);
R = 30;
D = 0;
[Ae,Be,De,Q_bar,R_bar] = MPC_martrix(A,B,C,D,Q,R,Np,Nc);
Yr = zeros(Np*4,1);
LimTheta = pi/4;
%约束
M = zeros(2,Nc);
M(1,1)=1;
M(2,1)=-1;
b = [LimTheta LimTheta]';


uall = MPC_optimize(Ae,Be,De,Q_bar,R_bar,Yr,err,M,b);
angle = uall(1,:);