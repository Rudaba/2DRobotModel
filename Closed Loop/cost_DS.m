function [integrand] = integralCost(x,u,xRef)

integrand     = 0.01*(x(1,1) - xRef(1,1)).^2 + 0.01*(x(2,1) - xRef(2,1)).^2 + 0.01*(x(3,1) - xRef(3,1)).^2 + 0.001* (u(1,1)-u(2,1)).^2;