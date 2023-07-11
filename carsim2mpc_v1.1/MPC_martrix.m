function [Ae,Be,De,Q_bar,R_bar] = MPC_martrix(A,B,C,D,Q,R,Np,Nc)
% 线性MPC, 输出扩张矩阵 Ae Be, 注意输入是增量形式，假设后半段du=0
    n=size(A,1);% A 是 n x n 矩阵, 得到 n
    p=size(B,2);% B 是 n x p 矩阵, 得到 p
    m=size(C,1);

    Ae=zeros(m*Np,n); % 每个元素m*p
    Be=zeros(m*Np,p*Nc); % 每个元素m*n
    De=zeros(m*Np,1); %干扰矩阵

    AA = makeAA(A,Np);%存储A^i次方

    for i=1:Np
        Ae((i-1)*m+1:i*m,1:n) = C*Apow(AA,i);
    end
%假设1
%     for i=1:Np
%         if(i==1)
%             De(1:i*m,1:4) = C*D;
%         else
%             De((i-1)*m+1:i*m,1:4) = C*Apow(AA,i-1)*D;
%         end
%     end
% 假设2
    for i=1:Np
        if(i==1)
            De(1:i*m,1) = C*D;
        else
            De((i-1)*m+1:i*m,1) = C*Apow(AA,i-1)*D + De((i-2)*m+1:(i-1)*m,1);
        end
    end

    for j=1:Nc
        for i=j:Np
            if(i==j)
                Be((i-1)*m+1:i*m,(j-1)*p+1:j*p) = C*B;
            else
                Be((i-1)*m+1:i*m,(j-1)*p+1:j*p) = C*Apow(AA,i-j)*B;
            end
        end
    end
%     Q_bar = kron(eye(Np),C*Q);
    Q_bar = Q;
    R_bar = kron(eye(Nc),R);
end

function AA = makeAA(A,Np)
    n=size(A,1);
    AA=zeros(n*Np,n);%存储A^i次方
    tmp = A;
    for i=1:Np
        AA((i-1)*n+1:i*n,1:n) = tmp;
        tmp = tmp*A;
    end
end

function Ai = Apow(AA,i)
    n = size(AA,2);
    Ai = AA((i-1)*n+1:i*n,1:n);
end

