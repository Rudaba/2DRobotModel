function [Mayer, Integral, dC_dx] = integralCost(x,u,xRef)

global MPCmodelNumber

Mayer = 0;

Qp = 1;
Qu = 10;

if MPCmodelNumber == 1
    
    Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);
    
    dC_dx = [Qp*x(1,:); ...
        Qp*x(2,:); ...
        Qp*x(3,:); ...
        Qu*u(1,:); ...
        Qu*u(2,:)];
    
    
elseif MPCmodelNumber == 2
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu * (u(1,:) - xRef(6,:)).^2 + Qu * (u(2,:) - xRef(7,:)).^2);
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:) - xRef(6,:)); ...
        Qu*(u(2,:) - xRef(7,:))];
    
end