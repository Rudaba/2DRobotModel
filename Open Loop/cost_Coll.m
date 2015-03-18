function [integrand] = integralCost(x,u,xRef)

% Over whole prediction horizon
% integrand1    = 3*(x(1,:) - xRef(1,:)).^2 + 3*(x(2,:) - xRef(2,:)).^2 + 1*(x(3,:) - xRef(3,:)).^2 + 0.0001* u(1,:).^2 + 0.0001* u(2,:).^2;
% integrand     = sum(integrand1);

Qp = 1;
Qu = 5;

% Over whole prediction horizon + yaw rate
integrand1    = Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:) - xRef(2,:)).^2 + Qp*(x(3,:) - xRef(3,:)).^2 + Qu*(u(1,:) - u(2,:)).^2;
integrand     = sum(integrand1);

% At the end of prediction horizon
% integrand1     = 5*(x(1,end) - xRef(1,end)).^2 + 5*(x(2,end) - xRef(2,end)).^2 + 10*(x(3,end) - xRef(3,end)).^2;
% integrand2     = 0.0001* u(1,:).^2 + 0.0001* u(2,:).^2;
% integrand      = integrand1 + sum(integrand2);

% % At the end of prediction horizon + yaw rate
% integrand1     = 2*(x(1,end) - xRef(1,end)).^2 + 2*(x(2,end) - xRef(2,end)).^2 + 5*(x(3,end) - xRef(3,end)).^2;
% integrand2     = 0.01*(u(1,:) - u(2,:)).^2;
% integrand      = sum(integrand1) + sum(integrand2);