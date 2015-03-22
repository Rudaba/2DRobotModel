function [integrand] = integralCost(x,u,xRef)

%Whole horizon and inputs
% integrand     = 3*(x(1,1) - xRef(1,1)).^2 + 3*(x(2,1) - xRef(2,1)).^2 + 1*(x(3,1) - xRef(3,1)).^2 + 0.0001* u(1,1).^2 + 0.0001* u(2,1).^2;

%Whole horizon and yaw rates
integrand     = 1*(x(1,1) - xRef(1,1)).^2 + 1*(x(2,1) - xRef(2,1)).^2 + 1*(x(3,1) - xRef(3,1)).^2 + 0.1*(u(1,1)-u(2,1)).^2;
% integrand     = 5*(x(3,1) - xRef(3,1)).^2 + 0.01*(u(1,1)-u(2,1)).^2;

