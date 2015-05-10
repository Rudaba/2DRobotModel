function [Mayer, Integral, dC_dx] = integralCost(x,u,v,psiDot,xRef)
global R b modelNumber

Mayer = 0;

% omegaR        = u(1,:);
% omegaL        = u(2,:);

% v             = R*(omegaR+omegaL)/2;

if modelNumber == 1
    
    Qp = 1;
    Qu = 1;
    Qs = 1;
%     Qv = 100;
    
%     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2 + Qs * (v - xRef(4,:)).^2 + Qv * (psiDot - xRef(5,:)).^2);
    Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2 + Qs * (v - xRef(4,:)).^2);
    
    dC_dx = [Qp*x(1,:); ...
        Qp*x(2,:); ...
        Qp*x(3,:); ...
        Qu*u(1,:) + Qs*(v - xRef(4,:))*R/2;...% + Qv*(psiDot - xRef(5,:))*R/(2*b); ...
        Qu*u(2,:) + Qs*(v - xRef(4,:))*R/2];% - Qv*(psiDot - xRef(5,:))*R/(2*b)];
    
    %     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qu*u(1,:); ...
    %         Qu*u(2,:)];
    
    
elseif modelNumber == 2
    
    Qp = 1;
    Qu = 1;
    Qs = 1;
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu*(u(1,:)-u(2,:)).^2) + Qs * (v - xRef(4,:)).^2;
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*R/2; ...
        Qu*(-1)*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*R/2];
    
end