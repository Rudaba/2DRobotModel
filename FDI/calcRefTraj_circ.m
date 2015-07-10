function refTraj = calcRefTraj_circ

%Calculate circular trajectory for 2D robot

R2D         = 180/pi;

thetaDot    = 10/R2D;
theta       = 0;
dt          = 0.1;
timeTaken   = length(0:dt:100);

R           = 10;

x(1)        = 5;
y(1)        = 0;
psi(1)      = theta;
V(1)        = R;
time(1)     = 0;
psiDot(1)   = thetaDot;

RR          = 2;
RL          = 2; 

[omegaR0,omegaL0] = calcFeedforward(V(1), psiDot(1),RR,RL);

omegaR(1) = omegaR0;
omegaL(1) = omegaL0;

for i = 1:timeTaken
    theta           = thetaDot * dt + theta;
    xDot            = R * cos(theta);
    x(i+1)          = x(i) + xDot*dt;
    yDot            = R * sin(theta);
    y(i+1)          = y(i) + yDot*dt;
    psi(i+1)        = theta;
    V(i+1)          = R;
    psiDot(i+1)     = thetaDot;
    time(i+1)       = time(i) + dt;
    
    [omegaR0,omegaL0] = calcFeedforward(V(i+1), psiDot(i+1),RR,RL);
    
    omegaR(i+1) = omegaR0;
    omegaL(i+1) = omegaL0;
end
% plot(x,y)
refTraj = [time;x;y;psi;V;psiDot;omegaR;omegaL]';
