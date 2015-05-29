function processLinearMPCData(x,y0,t0,tf,t_sort,N,intdt,m,n)
global refTraj

% Extract states and controls
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t                  = ((tf-t0)/2*t_sort+(tf+t0)/2);

omegaR0            = interp1(refTraj(:,1),refTraj(:,7),t,'pchip');
omegaL0            = interp1(refTraj(:,1),refTraj(:,8),t,'pchip');

uApp(:,1) = u(:,1) + omegaR0;
uApp(:,2) = u(:,2) + omegaL0;

[y0,tReal,yReal,uReal] = simulateRobotRK(y0,t,uApp,intdt,t0,tf);

%Plot results
figure
subplot(2,1,1)
plot(tReal,uReal(:,1),'-r')
hold on
subplot(2,1,2)
plot(tReal,uReal(:,2),'-b')
title('Controls Linear MPC, omegaR (top), omegaL (bottom)')

figure
subplot(2,1,1)
plot(tReal,yReal(:,1),'r')
hold on
% plot(t,y(:,1),'*g')
title('Integrated States, x (top), y (bottom)')
hold on
subplot(2,1,2)
plot(tReal,yReal(:,2),'b')
hold on
% plot(t,y(:,2),'*g')
% legend('integrated','optimal')


ref  = interp1(refTraj(:,1),refTraj(:,2:end),t,'pchip');

figure
plot(yReal(:,1),yReal(:,2),'b')
hold on
plot(ref(:,1),ref(:,2),'r')
title('x vs y')
legend('integrated','reference')