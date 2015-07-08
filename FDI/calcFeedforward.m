function [omegaR0,omegaL0] = calcFeedforward(V,psiDot)


%x = [0;0];

x = fsolve(@(x)vectorFF(x,V,psiDot),[0 0],optimset('Display','off'));

omegaR0 = x(1);
omegaL0 = x(2);

