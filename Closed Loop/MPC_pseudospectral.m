function [uMPC,tMPC] = MPC_pseudospectral(xlow,xupp,Flow,Fupp,iGfun,jGvar)

global m n N x Hp t0 t_sort

A = [];
iAfun = [];
jAvar = [];

[x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunPseudospectral', A, iAfun, jAvar, iGfun, jGvar);

% Extract states and controls from x vector
for j = 1:n
  y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end
for k = 1:m
  uMPC(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

tf   = t0 + Hp; 
tMPC = ((tf-t0)/2*t_sort+(tf+t0)/2);

% u  = uMPC(1,:)';
 