function refTraj = calcRefTraj_circ

%Calculate circular trajectory for 2D robot

R2D         = 180/pi;

thetaDot    = 10/R2D;
theta       = 0;
dt          = 0.1;
timeTaken   = length(0:dt:100);

R                   = 10;

x(1)                = 5;
y(1)                = 0;
psi(1)              = theta;
V(1)                = R;
time(1)             = 0;
psiDot(1)           = thetaDot;

[omegaR0,omegaL0]   = calcFeedforward([x(1),y(1),psi(1),V(1),psiDot(1)]);

omegaR0_store(1)    = omegaR0;
omegaL0_store(1)    = omegaL0;

for i = 1:timeTaken
    theta               = thetaDot * dt + theta;
    xDot                = R * cos(theta);
    x(i+1)              = x(i) + xDot*dt;
    yDot                = R * sin(theta);
    y(i+1)              = y(i) + yDot*dt;
    psi(i+1)            = theta;
    V(i+1)              = R;
    time(i+1)           = time(i) + dt;
    psiDot(i+1)         = thetaDot;
    [omegaR0,omegaL0]   = calcFeedforward([x(i+1),y(i+1),psi(i+1),V(i+1),psiDot(i+1)]);
    
    omegaR0_store(i+1)  = omegaR0;
    omegaL0_store(i+1)  = omegaL0;
end
% plot(x,y)
% title('Reference Trajectory')
% xlabel('x [m]')
% ylabel('y [m]')
refTraj = [time;x;y;psi;V;psiDot;omegaR0_store;omegaL0_store]';
