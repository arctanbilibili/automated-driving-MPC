function [E , H]=MPC_Matrices(A,B,Q,R,F,N)

n=size(A,1);% A 是 n x n 矩阵, 得到 n
p=size(B,2);% B 是 n x p 矩阵, 得到 p

%%%%%%%%%%%%
M=[eye(n);zeros(N*n,n)]; % 初始化 M 矩阵. M 矩阵是 (N+1)n x n的， 

% 它上面是 n x n 个 "I", 这一步先把下半部
% 分写成 0 

C=zeros((N+1)*n,N*p); % 初始化 C 矩阵, 这一步令它有 (N+1)n x NP 个 0
% 定义M 和 C 

tmp=eye(n); %定义一个n x n 的 I 矩阵
%　更新Ｍ和C

for i=1:N % 循环，i 从 1到 N
rows =i*n+(1:n); %定义当前行数，从i x n开始，共n行 
C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %将c矩阵填满
tmp= A*tmp; %每一次将tmp左乘一次A
M(rows,:)=tmp; %将M矩阵写满
end

% 定义Q_bar和R_bar
Q_bar = kron(eye(N),Q);  % 张量积 kron([1 0;0 1],[1 0;0 2]) = [1     0     0     0 ; 0     2     0     0 ;0     0     1     0 ;0     0     0     2]
Q_bar = blkdiag(Q_bar,F);% blkdiag(A,B) = [A 0;0 B]
R_bar = kron(eye(N),R);

% 计算G, E, H
G=M'*Q_bar*M; % G: n x n
E=C'*Q_bar*M; % E: NP x n

H=C'*Q_bar*C+R_bar; % NP x NP 
H=(H+H')/2;
