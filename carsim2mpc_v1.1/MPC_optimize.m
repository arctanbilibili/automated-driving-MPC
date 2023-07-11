function uall = MPC_optimize(Ae,Be,G,Q_bar,R_bar,Yr,xk,M,b)
% 线性MPC, 输出扩张矩阵 Ae Be, 注意输入是增量形式，假设后半段du=0
    H = Be'*Q_bar*Be+R_bar;
    F = -Be'*Q_bar*(Yr-Ae*xk-G);
    
    options = optimoptions( 'quadprog' , 'Algorithm' , 'active-set' );
    uall = zeros(size(R_bar,1),1);
    uall = quadprog(H,F,M,b,[],[],[],[],uall,options);
    %u = U_k(1:2,1); % 取第一个结果
end
