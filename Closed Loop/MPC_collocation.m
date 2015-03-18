function [uMPC,tMPC] = MPC_collocation(xlow,xupp,Flow,Fupp,iGfun,jGvar)

global m n N x Hp t0

A = [];
iAfun = [];
jAvar = [];

[x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunCollocation', A, iAfun, jAvar, iGfun, jGvar);

% Extract states and controls from x vector
for j = 1:n
  y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end
for k = 1:m
  uMPC(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

tMPC = t0 : Hp/N : t0+Hp;

% u = uMPC(1,:)';
 