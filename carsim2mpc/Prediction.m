function u_k= Prediction(x_k,E,H,N,p)

% lb = -100*ones(size(H,1),p);%下限
% ub = 300*ones(size(lb));

% lb = [];
% ub = [];
% lb = zeros(3,1);
% ub = ones(size(lb));
ss = size(H);
n = ss(1);
lb = -pi/4*ones(n,1);
ub = pi/4*ones(n,1);
% lb = [];
% ub = [];


options = optimoptions( 'quadprog' , 'Algorithm' , 'active-set' );
U_k = zeros(N*p,1); % NP x 1
U_k = quadprog(H,E*x_k,[],[],[],[],lb,ub,U_k,options);
u_k = U_k(1:p,1); % 取第一个结果

end