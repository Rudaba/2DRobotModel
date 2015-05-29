function [tReal, yReal, u] = processCollData(x,y0,t0,tf,N,intdt,m,n)

% Extract states and controls
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

dt   = (tf - t0) / (N);
t    = [t0:dt:tf];

[y0,tReal,yReal] = simulateRobotRK(y0,t,u,intdt,t0,tf);

% %Plot results
% figure
% subplot(2,1,1)
% title('Controls Coll, omegaR (top), omegaL (bottom)')
% plot(t,u(:,1),'-r')
% hold on
% subplot(2,1,2)
% plot(t,u(:,2),'-b')
% 
% figure
% subplot(2,1,1)
% plot(tReal,yReal(:,1),'r')
% hold on
% plot(t,y(:,1),'*g')
% title('Integrated States, x (top), y (bottom)')
% hold on
% subplot(2,1,2)
% plot(tReal,yReal(:,2),'b')
% hold on
% plot(t,y(:,2),'*g')
% legend('integrated','optimal')
% 
% figure
% plot(yReal(:,1),yReal(:,2),'b')
% hold on
% plot(y(:,1),y(:,2),'*g')
% title('x vs y')
% legend('integrated','optimal')