function [plantData, y0, tReal, yReal, uReal] = integrateStates(x,y0,t0,t_sort,N,intdt,m,n,model,refTraj)
global Hp MPCUpdateRate

tf = t0 + MPCUpdateRate;

% Extract states and controls
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t                  = (((t0+Hp)-t0)/2*t_sort+((t0+Hp)+t0)/2);

if model == 1
    
    omegaR0               = interp1(refTraj(:,1),refTraj(:,7),t);
    omegaL0               = interp1(refTraj(:,1),refTraj(:,8),t);
    
    uApp(:,1) = u(:,1) + omegaR0;
    uApp(:,2) = u(:,2) + omegaL0;
    
    [y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,uApp,intdt,t0,tf);
    
elseif model == 2
    
    [y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,u,intdt,t0,tf);
    
elseif model == 3
    
    [y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,y(:,4:5),intdt,t0,tf);
    
end

yrefstore               = interp1(refTraj(:,1),refTraj(:,2:4),tReal)';

plantData.y      = yReal';
plantData.t      = tReal';
plantData.u      = uReal';
plantData.yref   = yrefstore;

u                = interp1(tReal(:,1),uReal(:,1:2),t0,'pchip')';
u                = u + 0.01*rand(2,1);

measurement      = interp1(tReal(:,1),yReal(:,1:3),t0,'pchip')';
measurement      = measurement + 0.01*rand(3,1);
