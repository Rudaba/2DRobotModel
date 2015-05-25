data = load('plantData_LMPC');

time = [data.plant_tout];
y    = [data.plant_yout];
u    = [data.plant_uout];
yref = [data.plant_yrefout];

dt   = 1;

figure
plot(time,yref(1,:),'r')
hold on
plot(time,y(1,:),'b')
title('x')

figure
plot(time,yref(2,:),'r')
hold on
plot(time,y(2,:),'b')
title('y')

figure
plot(time,yref(3,:),'r')
hold on
plot(time,y(3,:),'b')
title('psi')

figure
% plot(time(1,1:dt:end),round(u(1,1:dt:end)*100)/100)
plot(time(1,1:dt:end),u(1,1:dt:end))
title('OmegaR')

figure
% plot(time(1,1:dt:end),round(u(2,1:dt:end)*100)/100)
plot(time(1,1:dt:end),u(2,1:dt:end))
title('OmegaL')

figure
plot(yref(1,:),yref(2,:),'r')
hold on
plot(y(1,:),y(2,:),'b')
title('Trajectory')

