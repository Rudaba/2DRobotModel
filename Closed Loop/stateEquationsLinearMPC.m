function [yDots, vOut, psiDot, df1_dx, df2_dx, df3_dx]  = stateEquationsLinearMPC(stateVec,u,xRef)

global n N R b

yDots               = zeros(n, length(stateVec));
v                   = zeros(n,1);
psi                 = zeros(n,1);


for i = 1:length(xRef)
    [omegaR0,omegaL0]   = calcFeedforward(xRef(:,i));
    [A, B, V, PSI]      = Statespace(xRef(:,i),omegaR0,omegaL0);
    yDots(:,i)          = A*stateVec(i,:)' + B*u(i,:)';
    v(i,:)              = V;
    vOut(i,:)           = R/2*(omegaR0 + u(i,1)) + R/2*(omegaL0 + u(i,2));
    psiDot(i,:)         = R/(2*b)*(omegaR0 + u(i,1)) - R/(2*b)*(omegaL0 + u(i,2));
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

