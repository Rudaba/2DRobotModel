function [integrand] = integralCost(x,u,xRef)

Qu = 0.1;
Qp = 0.01;

integrand     = Qp*(x(1,1) - xRef(1,1)).^2 + Qp*(x(2,1) - xRef(2,1)).^2 + Qp*(x(3,1) - xRef(3,1)).^2 + Qu*(u(1,1)-u(2,1)).^2;