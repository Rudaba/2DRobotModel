function [Mayer, Integral, dC_dx] = integralCost(x,u,xRef)

Mayer = 0;

Qp = 2;
Qu = 4;

Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu*(u(1,:)-u(2,:)).^2);

if nargout > 2
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:)-u(2,:)); ...
        Qu*(-1)*(u(1,:)-u(2,:))];
end
