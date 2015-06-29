function [y0,tReal,yReal,uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,model,refTraj)
global Hp
% Extract states and controls
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t                  = (((t0+Hp)-t0)/2*t_sort+((t0+Hp)+t0)/2);

if model == 1
    
     xRef               = interp1(refTraj(:,1),refTraj(:,2:end),t);
     
     for i = 1:length(t)
         [omegaR0(i),omegaL0(i)]   = calcFeedforward(xRef(i,4),xRef(i,5));
     end
    
    uApp(:,1) = u(:,1) + omegaR0';
    uApp(:,2) = u(:,2) + omegaL0';
    
    [y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,uApp,intdt,t0,tf);

elseif model == 2
    
    [y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,u,intdt,t0,tf);

end

% t                  = (((t0+Hp)-t0)/2*t_sort+((t0+Hp)+t0)/2);
% y0 = interp1(t,y,tf,'pchip')';

