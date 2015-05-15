data = load('plantData');

time = [data.plant_tout];
y    = [data.plant_yout];
u    = [data.plant_uout];
yref = [data.plant_yrefout];

figure
plot(time,y(1,:),'b')
hold on
plot(time,yref(1,:),'r')
title('x')

figure
plot(time,y(2,:),'b')
hold on
plot(time,yref(2,:),'r')
title('y')

figure
plot(time,y(3,:),'b')
hold on
plot(time,yref(3,:),'r')
title('psi')

figure
plot(time,u(1,:))
title('OmegaR')

figure
plot(time,u(2,:))
title('OmegaL')
