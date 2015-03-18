function [uMPC,tMPC] = MPC_directShooting(xlow,xupp,Flow,Fupp,iGfun,jGvar)

global x m N t0 Hp

A = [];
iAfun = [];
jAvar = [];
%y0 = xNav;

[x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunDirectShooting', A, iAfun, jAvar, iGfun, jGvar);

for k = 1:m
  uMPC(:,k) = x((k-1)*(N+1)+1:k*(N+1));
end

tMPC = t0:Hp/N:t0+Hp;

% u = uMPC(1,:)';
 