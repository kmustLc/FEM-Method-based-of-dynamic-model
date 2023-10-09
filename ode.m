%-----------------------
% 柔性扭转模型，有限元法
%-----------------------
clc
clear
close all

%-1.1--试验台基本参数
La = 53e-3;
d = 25e-3;
G = 8.3e13;
rho = 7500;              % 轴的密度 kg.m3
n = 10;                  % 单元数
L = La/n;                % 单元轴段长度
I = (pi*d^4/32)*rho*L/3; % 轴段转动惯量
ke = G/L*(pi*d^4/32);    % 刚度
cs = 0.0036;             % 结构扭转阻尼
Me = [I I/2;
      I/2 I];            % 单元等效质量矩阵
Ke = [ke -ke
     -ke ke];            % 单元扭转刚度矩阵
Cse = [cs -cs
      -cs cs];           % 单元扭转阻尼矩阵

CeA = 2.8e-3;            % 4 节点轴承转动阻尼
CeB = 2.8e-3;            % 9 节点轴承转动阻尼
CeR = 7e-4;              % 待测轴承 7节点轴承转动阻尼

%-1.2--扭矩扰动设置
param.t = 0:0.00001:10;
% Tm = 0.74 + 0.37*param.t;      % 时变电机扭矩
% Tm = 0.74 ;                    % 时不变电机扭矩 
% omega = Tm/cs;
% theta = omega*t;
% theta0 = 2*pi/5.7;              % 扰动角度间隔
% theta = mod(theta,theta0);
% Ltheta = 20/360*2*pi;
% theta_a = 0;
% theta_b = theta0/6;
% theta_c = theta0/6+2*theta0/3;
% theta_d = theta0;
% Ap = 0.0074;
% P = zeros(length(theta),1);
% for i = 1:length(theta)
% if  theta(i)>=theta_a && theta(i)<=theta_b
%     P(i) = Ap*sin(pi/Ltheta*(theta(i)-theta_a+Ltheta/2));
% elseif theta(i)>theta_b && theta(i)<=theta_c
%     P(i) = 0;
% elseif theta(i)>theta_c && theta(i)<theta_d
%     P(i) = Ap*sin(pi/Ltheta*(theta(i)-theta_c));
% end
% end
% P =(P.*(P>0)).' ;

%-2--总体矩阵组装

%-2.1--总体质M量矩阵组装
M_temp = zeros(n+1,n+1);
M = M_temp;
for i=1:n
    j = i+1;
    temp = massMatrixAssemble(M_temp,Me,i,j);
    M = M + temp;
end

%-2.2--总体刚度矩阵组装
K_temp = zeros(n+1,n+1);
K = K_temp;
for i=1:n
    j = i+1;
    temp = stiffnessMatrixAssemble(K_temp,Ke,i,j);
    K = K + temp;
end

%-2.3--总体阻尼矩阵组装
C_temp = zeros(n+1,n+1);
C = C_temp;
for i=1:n
    j = i+1;
    temp = dampingMatrixAssemble(C_temp,Cse,i,j);
    C = C + temp;
end
C(4,4) = C(4,4)+CeA;
C(7,7) = C(7,7)+CeR;
C(9,9) = C(9,9)+CeB;



%-2.4--外力矩阵
% Fext = zeros(11,1);
% Fext(11,:) = Tm;
% 
% Gp = [0 0 0 0 0 0 1 0 0 0 0].'; % 扰动配置向量
% Perturbation = Gp*P;

%-3.1--系统动力学微分方程参数
I = eye(11);
zero = zeros(11);

param.A = [I zero; zero inv(M)]*[zero I;-K -C];

param.B = [I zero; zero inv(M)];

%% 
%-3--微分方程及其求解

%-3.2--系统动力学微分方程求解
X0 = zeros(22,1);

theta_period = 2*pi*10;

tspan = 0:theta_period;

opts=odeset('RelTol',1e-6);

[outputTheta,outputQ] = ode15s(@systemDynamic,tspan,X0,[],param); %微分方程组的数值解

 
%-3.3--系统动力学微分方程

function QDot = systemDynamic(theta,X0,param)

theta0 = 2*pi/5.7;              % 扰动角度间隔
theta = mod(theta,theta0);
Ltheta = 20/360*2*pi;
theta_a = 0;
theta_b = theta0/6;
theta_c = theta0/6+2*theta0/3;
theta_d = theta0;
Ap = 0.0074;
P = zeros(length(theta),1);

if  theta>=theta_a && theta<=theta_b
    P = Ap*sin(pi/Ltheta*(theta-theta_a+Ltheta/2));
elseif theta>theta_b && theta<=theta_c
    P = 0;
elseif theta>theta_c && theta<theta_d
    P = Ap*sin(pi/Ltheta*(theta-theta_c));
end

P =(P.*(P>0)).' ;

Tm = 0.74;
% Tm = 0.74 + 0.37*param.t;      % 时变电机扭矩

Fext = zeros(11,1);

Fext(11,:) = Tm;

Gp = [0 0 0 0 0 0 1 0 0 0 0].'; % 扰动配置向量

Perturbation = Gp*P;

Q = X0(1:22);

QDot = (param.A*Q + param.B*[zeros(11,1);Fext+Perturbation]);

end






