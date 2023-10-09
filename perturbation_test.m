clc
clear
close all

%-1.2--扭矩扰动设置
t = 0:0.00001:10;

% Tm = 0.74 + 0.37*t;      % 时变电机扭矩
Tm = 0.74 ;              % 时不变电机扭矩
cs = 0.0036;   
omega = Tm/cs;
theta0 = 2*pi/5.7;              % 扰动角度间隔
theta = omega*t;
theta = mod(theta,theta0);
Ltheta = 20/360*2*pi;
theta_a = 0;
theta_b = theta0/6;
theta_c = theta0/6+2*theta0/3;
theta_d = theta0;
Ap = 0.0074;
P = zeros(length(theta),1);
for i = 1:length(theta)
if  theta(i)>=theta_a && theta(i)<=theta_b
    P(i) = Ap*sin(pi/Ltheta*(theta(i)-theta_a+Ltheta/2));
elseif theta(i)>theta_b && theta(i)<=theta_c
    P(i) = 0;
elseif theta(i)>theta_c && theta(i)<theta_d
    P(i) = Ap*sin(pi/Ltheta*(theta(i)-theta_c));
end
end
P = P.*(P>0);
plot(omega*t,P)
y = P;
l = length(y);
n = 0:l;
fs = 54;
[Yf1,f1]=Time_frequency(y, fs);
figure(2)
set(gcf,'Units','centimeter','Position',[5 5 12 4]);
plot(f1,Yf1,'LineWidth',0.8),
xlim([1 10])
set(gca,'FontSize',13,'Fontname', 'Times New Roman','LineWidth',1)
xlabel('角度频率 (Ev/Rev)','FontSize',13,'Fontname','宋体');
ylabel('Acc','FontSize',13,'Fontname','宋体');


