function [omegaR0,omegaL0] = calcFeedforward(xRef)


%x = [0;0];

x = fsolve(@(x)vectorFF(x,xRef),[0 0],optimset('Display','off'));

omegaR0 = x(1);
omegaL0 = x(2);

