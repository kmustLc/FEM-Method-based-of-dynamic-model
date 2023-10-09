clc
clear
close all
% ----------积分区间------------ %
theta_period = 2*pi*300;
theta = 0:1e-2:theta_period;

% ------------初始值------------ %
q0 = [0,0,0,0,0,0,0,0,0,0,0,1e-5,0,0,0,0,0,0,0,0,0,0,0];

% !--------求解微分方程--------! %
opts=odeset('RelTol',1e-6);
tic
[T,x] = ode15s('ode',theta,q0,opts);
toc;