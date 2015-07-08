function [yDots, dv, dpsiDot, df1_dx, df2_dx, df3_dx]  = stateEquationsLinearMPC(stateVec, u, t, tRef, xRef)

global n N R b

yDots               = zeros(n, length(stateVec));
v                   = zeros(n,1);
psi                 = zeros(n,1);

R = 2;
b = 1;


for i = 1:length(xRef)
    omegaR0                     = xRef(6,i);
    omegaL0                     = xRef(7,i);
    omegaR                      = omegaR0 + u(i,1);
    omegaL                      = omegaL0 + u(i,2);
    [A,B,V0,PSI0,PSI_DOT0]      = Statespace(xRef(:,i),omegaR0,omegaL0);
    psiDot                      = R*(omegaR - omegaL) / (2*b);
    v                           = R*(omegaR + omegaL) / (2);
    dpsiDot(i,:)                = psiDot - PSI_DOT0;
    dv(i,:)                     = v - V0;
    yDots(:,i)                  = A*stateVec(i,:)' + B*u(i,:)';
    psi(i,:)                    = PSI0; 
    v0(i,:)                     = V0;
  
end

df1_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (-v0.*sin(psi))';
    (R/2.*cos(psi))';
    (R/2.*cos(psi))'];

df2_dx = [zeros(1,N+1);
    zeros(1,N+1);
    (v0.*cos(psi))';
    (R/2.*sin(psi))';
    (R/2.*sin(psi))'];

df3_dx = [zeros(1,N+1);
    zeros(1,N+1);
    zeros(1,N+1);
    R/(2*b)*ones(1,N+1);
    -R/(2*b)*ones(1,N+1)];

