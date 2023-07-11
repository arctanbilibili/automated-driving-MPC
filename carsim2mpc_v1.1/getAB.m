function [A,B] = getAB(vx,a,b,m,cf,cr,Iz,Ts)%计算离散的A B矩阵
%   连续转离散 vx必须要转换到 >0 
%   Detailed explanation goes here
% cf=-80000;
% cr=cf;
% m=1413;
% Iz=1536.7;%yaw惯量
% a=1.015;
% b=2.910-1.015;
if(vx < 0.01)
    vx=0.01; 
end
Ac=[0,1,0,0;
    0,(cf+cr)/(m*vx),-(cf+cr)/m,(a*cf-b*cr)/(m*vx);
    0,0,0,1;
    0,(a*cf-b*cr)/(Iz*vx),-(a*cf-b*cr)/Iz,(a*a*cf+b*b*cr)/(Iz*vx)];
Bc=[0;
    -cf/m;
    0;
    -a*cf/Iz];

%离散化为A B矩阵
A=zeros(4,4);
B=zeros(4,1);
coder.extrinsic('c2dm');
[A1,B1,~,~] = c2dm(Ac,Bc,[],[],Ts);
A=A1;
B=B1;


end