
X0 = zeros(22,1); %初始值，包括位置和速度
param.A = M; %将质量矩阵
param.B = C; %阻尼矩阵
param.C = K; %刚度矩阵 
tspan = linspace(0,10,100);
[tlist,Xlist] = ode45(@Ode_45,tspan,X0,[],param); %微分方程组的数值解
subplot(2,1,1)
plot(tlist,Xlist(:,1),"*");
title("x")
subplot(2,1,2)
plot(tlist,Xlist(:,2),"*");
title("y")
%微分方程组的表达式
function DStateVarDt=Ode_45(t,X0,param)
ii = length(X0);
X = X0(1:(ii/2)); %前一半是X
XDot = X0((ii/2+1):ii); %后一半是X关于时间的导数
% Tm = 0.74;
Tm = 0.74 + 0.37*t;      % 时变电机扭矩
Fext = zeros(11,1);
Fext(11,1) = Tm;
XDotDot = inv(param.A)*(-param.B*XDot-param.C*X+Fext);%降阶后的表达式
DStateVarDt = [XDot; XDotDot];
end