function [Mayer, Integral, dC_dx] = Cost(x,u)

Mayer       = 0;
% 
% Integral    = 1*x(1,:).^2+ 1*x(2,:).^2 + 1*x(3,:).^2 + 0.01*u(1,:).^2 + 0.01*u(2,:).^2;
% 
% Mayer = 0;

Qp = 10;
Qu = 2;

Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);

% Integral = 10*(x(3,:)-xRef(3,:)).^2+ 0.01*(u(1,:)-u(2,:)).^2;

% Integral = 1*(x(3,:)-xRef(3,:)).^2 + 0.1*(x(4,:)-xRef(5,:)).^2 + 0.1*(u(1,:)-u(2,:)).^2;

dC_dx = [Qp*(x(1,:)); ...
         Qp*(x(2,:)); ...
         Qp*(x(3,:)); ...
         Qu*(u(1,:)); ...
         Qu*(u(2,:))];
