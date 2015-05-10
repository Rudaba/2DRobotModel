function [yDots, v, df1_dx, df2_dx, df3_dx]  = stateEquationsNMPC(stateVec,u,t)

global n N R b

yDots = zeros(n, length(t));

%Extract Data
psi           = stateVec(:,3);
y             = stateVec(:,2);
x             = stateVec(:,1);

omegaR        = u(:,1);
omegaL        = u(:,2);

v             = R*(omegaR+omegaL)/2;

%Solve DE's
psiDot        = R*(omegaR-omegaL)/(2*b);

yDot        = v.*sin(psi);
xDot        = v.*cos(psi);

yDots(1,:)  = xDot;
yDots(2,:)  = yDot;
yDots(3,:)  = psiDot;



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

