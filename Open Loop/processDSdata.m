function processDSdata(x,y0,t0,tf,Nu,intdt,m)

%Extract inputs and states
dtu = (tf - t0) / (Nu);
tu  = [t0:dtu:tf];

for k = 1:m
    u(:,k) = x((k-1)*(Nu+1)+1:k*(Nu+1));
end

[y0,tReal,yReal] = simulateRobotRK(y0,tu,u,intdt,t0,tf);

%Plot results
figure
subplot(2,1,1)
title('Controls DS, omegaR (top), omegaL (bottom)')
plot(tu,u(:,1),'-r')
hold on
subplot(2,1,2)
plot(tu,u(:,2),'-b')

figure
subplot(2,1,1)
plot(tReal,yReal(:,1),'r')
title('Integrated States, x (top), y (bottom)')
hold on
subplot(2,1,2)
plot(tReal,yReal(:,2),'b')

figure
plot(yReal(:,1),yReal(:,2),'g')
title('x vs y')