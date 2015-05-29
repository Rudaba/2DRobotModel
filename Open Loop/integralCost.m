function [Mayer, Integral, dC_dx] = integralCost(x,u,xRef)

global MPCmodelNumber 

Mayer = 0;

if MPCmodelNumber == 5
    
    Qp = 1;
    Qu = 1;
    
    Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);
    
    dC_dx = [Qp*x(1,:); ...
        Qp*x(2,:); ...
        Qp*x(3,:); ...
        Qu*u(1,:); ...
        Qu*u(2,:)];
    
    
else 
    
    Qp = 1;
    Qu = 1;
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu * (u(1,:) - xRef(6,:)).^2 + Qu * (u(2,:) - xRef(7,:)).^2);
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:) - xRef(6,:)); ...
        Qu*(u(2,:) - xRef(7,:))];
    
end