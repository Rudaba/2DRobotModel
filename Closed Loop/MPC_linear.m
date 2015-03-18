function [uMPC,tMPC] = MPC_linear(xlow,xupp,Flow,Fupp,iGfun,jGvar)

global m n N x Hp t0 t_sort refTraj

A = [];
iAfun = [];
jAvar = [];

[x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunLMPCSNOPT', A, iAfun, jAvar, iGfun, jGvar);

% Extract states and controls from x vector
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end
for k = 1:m
    uMPC(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

tf   = t0 + Hp;
tMPC = ((tf-t0)/2*t_sort+(tf+t0)/2);

xRef                      = interp1(refTraj(:,1),refTraj(:,2:end),tMPC);

for i = 1:length(tMPC)
    [omegaR0,omegaL0]   = calcFeedforward(xRef(i,:));
    
    uMPC(i,1)    = uMPC(i,1) + omegaR0;
    uMPC(i,2)    = uMPC(i,2) + omegaL0;
    
end
