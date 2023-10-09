function y = massMatrixAssemble(M,m,i,j)

M(i,i) = M(i,i) + m(1,1);
M(i,j) = M(i,j) + m(1,2);
M(j,i) = M(j,i) + m(2,1);
M(j,j) = M(j,j) + m(2,2);
y = M;
end