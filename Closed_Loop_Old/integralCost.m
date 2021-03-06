function [Mayer, Integral, dC_dx] = integralCost(x,u,v,psiDot,xRef)
global R b modelNumber

Mayer = 0;

if modelNumber == 1
    
    Qp = 5;
    Qv = 10;
    Qs = 10;
    
    Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qv * (v - xRef(4,:)).^2 + Qs * (psiDot - xRef(5,:)).^2);
    
    dC_dx = [Qp*x(1,:); ...
        Qp*x(2,:); ...
        Qp*x(3,:); ...
        Qv*(v - xRef(4,:))*R/2 + Qs*(psiDot - xRef(5,:))*R/(2*b); ...
        Qv*(v - xRef(4,:))*R/2 + Qs*(-1)*(psiDot - xRef(5,:))*R/(2*b)];
    
elseif modelNumber == 2
    
    Qp = 5;
    Qv = 10;
    Qs = 10;
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qv * (v - xRef(4,:)).^2 + Qs * (psiDot - xRef(5,:)).^2);
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qv*(v - xRef(4,:))*R/2 + Qs*(psiDot - xRef(5,:))*R/(2*b); ...
        Qv*(v - xRef(4,:))*R/2 + Qs*(-1)*(psiDot - xRef(5,:))*R/(2*b)];
    
end