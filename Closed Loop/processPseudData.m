function processPseudData(x,y0,t0,tf,t_sort,N,intdt,m,n)
global refTraj

% Extract states and controls
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t                  = ((tf-t0)/2*t_sort+(tf+t0)/2);

[y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,u,intdt,t0,tf);

%Plot results
figure
subplot(2,1,1)
title('Controls Pseudospectral, omegaR (top), omegaL (bottom)')
plot(tReal,uReal(:,1),'-r')
hold on
subplot(2,1,2)
plot(tReal,uReal(:,2),'-b')

figure
subplot(2,1,1)
plot(tReal,yReal(:,1),'r')
hold on
plot(t,y(:,1),'*g')
title('Integrated States, x (top), y (bottom)')
hold on
subplot(2,1,2)
plot(tReal,yReal(:,2),'b')
hold on
plot(t,y(:,2),'*g')
legend('integrated','optimal')

ref  = interp1(refTraj(:,1),refTraj(:,2:end),tReal,'pchip');
figure
plot(yReal(:,1),yReal(:,2),'b')
hold on
plot(ref(:,1),ref(:,2),'r')
title('x vs y')
legend('integrated','referece')