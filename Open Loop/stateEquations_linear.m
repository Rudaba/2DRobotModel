function [yDots, df1_dx, df2_dx, df3_dx] = stateEquations_linear(stateVec, u, xRef)

global n N

yDots               = zeros(n, length(stateVec));
v                   = zeros(n,1);
psi                 = zeros(n,1);

%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres


for i = 1:length(xRef)
    [omegaR0,omegaL0]   = calcFeedforward(xRef(:,i));
    [A, B, V, PSI]      = Statespace(xRef(:,i),omegaR0,omegaL0);
    yDots(:,i)          = A*stateVec(i,:)' + B*u(i,:)';
    v(i,:)              = V;
    psi(i,:)            = PSI; 
end

df1_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (-v.*sin(psi))';
    (R/2.*cos(psi))';
    (R/2.*cos(psi))'];

df2_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (v.*cos(psi))';
    (R/2.*sin(psi))';
    (R/2.*sin(psi))'];

df3_dx = [zeros(1,N+1);
    zeros(1,N+1);
    zeros(1,N+1);
    R/(2*b)*ones(1,N+1);
    -R/(2*b)*ones(1,N+1)];

