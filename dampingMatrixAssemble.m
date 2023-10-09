function y = dampingMatrixAssemble(C,c,i,j)

C(i,i) = C(i,i) + c(1,1);
C(i,j) = C(i,j) + c(1,2);
C(j,i) = C(j,i) + c(2,1);
C(j,j) = C(j,j) + c(2,2);
y = C;
end