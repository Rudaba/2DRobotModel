function [yDots, df1_dx, df2_dx, df3_dx] = stateEquations_linear(stateVec, u, xRef, RR, RL)

global n N 

yDots               = zeros(n, length(stateVec));
v                   = zeros(n,1);
psi                 = zeros(n,1);

%Robot Constants
b       = 1; %Distance between centre of tyres


for i = 1:length(xRef)
    [omegaR0,omegaL0]   = calcFeedforward(xRef(4,i),xRef(5,i),RR,RL);
    [A, B, V, PSI]      = Statespace(xRef(:,i),omegaR0,omegaL0);
    yDots(:,i)          = A*stateVec(i,:)' + B*u(i,:)';
    v(i,:)              = V;
    psi(i,:)            = PSI; 
end

df1_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (-v.*sin(psi))';
    (RR/2.*cos(psi))';
    (RL/2.*cos(psi))'];

df2_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (v.*cos(psi))';
    (RR/2.*sin(psi))';
    (RL/2.*sin(psi))'];

df3_dx = [zeros(1,N+1);
    zeros(1,N+1);
    zeros(1,N+1);
    RR/(2*b)*ones(1,N+1);
    -RL/(2*b)*ones(1,N+1)];

