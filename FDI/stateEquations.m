function [yDots, df1_dx, df2_dx, df3_dx]  = stateEquations(stateVec,u,t,RR,RL)

global n N 

yDots = zeros(n, length(t));

%Robot Constants
b       = 1; %Distance between centre of tyres

%Extract Data
psi           = stateVec(:,3);
y             = stateVec(:,2);
x             = stateVec(:,1);

omegaR        = u(:,1);
omegaL        = u(:,2);

v             = (RR*omegaR + RL*omegaL)/2;

%Solve DE's
psiDot        = (RR*omegaR - RL*omegaL)/(2*b);

yDot        = v.*sin(psi);
xDot        = v.*cos(psi);

yDots(1,:)  = xDot;
yDots(2,:)  = yDot;
yDots(3,:)  = psiDot;

if nargout > 1
    
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
end
