clear all;
clc;
%几何定义
node=[1  0  0;
            2  0  0;
            3  0  0];                     %节点编号及坐标
element=[1 2;2 3];                %定义单元
%-----------------------------------
dof=length(node(1,:));         %总体自由度数
k=[100 200];                         %各单元刚度系数
K=zeros(dof);                       %整体刚度矩阵
u=ones(dof,1)*1e6 ;             %位移列矩阵
f=ones(dof,1)*1e6;               %节点力列矩阵
ele_num=length(element(1,:));       %单元个数
for i=1:ele_num
    K=assemStiffness(K,k(i),element(i,1),element(i,2));
 end
%-----------定义边界条件------------------
u(1)=0;
f(2)=0;
f(3)=15;
%求解未知自由度
index=[];      %未知自由度的索引
p=[];          %未知自由度对应的节点力矩阵
for i=1:dof
    if u(i)~=0
        index=[index,i];
        p=[p;f(i)];
    end
end
u(index)=K(index,index)\p;    %高斯消去未知节点位移计算
f=K*u;                                  %节点力计算


function K =assemStiffness(K,k,i,j)
%assemStiffness This function assembles the element stiffness
% matrix k of the spring with nodes i and j into the
% global stiffness matrix K.
% This function returns the global stiffness matrix K
% after the element stiffness matrix ke is assembled.
ke=[k -k;-k k];
K(i,i) = K(i,i) + ke(1,1);
K(i,j) = K(i,j) + ke(1,2);
K(j,i) = K(j,i) + ke(2,1);
K(j,j) = K(j,j) + ke(2,2);
end














