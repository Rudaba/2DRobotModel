function processDSdata(x,y0,t0,Hp,Nu,intdt,m)
global refTraj

tf  = t0 + Hp; 

%Extract inputs and states
dtu = (tf - t0) / (Nu);
tu  = [t0:dtu:tf];

for k = 1:m
    u(:,k) = x((k-1)*(Nu+1)+1:k*(Nu+1));
end

[y0,tReal,yReal,uReal] = simulateRobotRK(y0,tu,u,intdt,t0,tf);

%Plot results
figure
subplot(2,1,1)
title('Controls DS, omegaR (top), omegaL (bottom)')
plot(tReal,uReal(:,1),'-r')
hold on
subplot(2,1,2)
plot(tReal,uReal(:,2),'-b')

figure
subplot(2,1,1)
plot(tReal,yReal(:,1),'r')
title('Integrated States, x (top), y (bottom)')
hold on
subplot(2,1,2)
plot(tReal,yReal(:,2),'b')

ref  = interp1(refTraj(:,1),refTraj(:,2:end),tReal,'pchip');
figure
plot(yReal(:,1),yReal(:,2),'b')
hold on
plot(ref(:,1),ref(:,2),'r')
title('x vs y')
legend('Integrated','Optimal')
