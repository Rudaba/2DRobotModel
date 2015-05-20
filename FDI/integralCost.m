function [Mayer, Integral, dC_dx] = integralCost(x,u,v,psiDot,xRef,RR,RL,t)
global modelNumber b

Mayer = 0;

% omegaR        = u(1,:);
% omegaL        = u(2,:);

% v             = R*(omegaR+omegaL)/2;

if modelNumber == 1
    
    Qp = 1;
    Qu = 1;
    Qs = 1;
    %     Qv = 100;
    
    
    %     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2 + Qs * (v - xRef(4,:)).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qu*u(1,:) + Qs*(v - xRef(4,:))*R/2; ...
    %         Qu*u(2,:) + Qs*(v - xRef(4,:))*R/2];
    %
    %     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2 + Qs * (v - xRef(4,:)).^2 + Qv * (psiDot - xRef(5,:)).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qu*u(1,:) + Qs*(v - xRef(4,:))*R/2 + Qv*(psiDot - xRef(5,:))*R/(2*b); ...
    %         Qu*u(2,:) + Qs*(v - xRef(4,:))*R/2 - Qv*(psiDot - xRef(5,:))*R/(2*b)];
    
    %     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qu*u(1,:); ...
    %         Qu*u(2,:)];
    %
    
    %     Integral    = 0.5*(Qp*x(1,:).^2+ Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*(u(1,:) - u(2,:)).^2 + Qs * (v - xRef(4,:)).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qu*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*R/2; ...
    %         Qu*(-1)*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*R/2];
    
elseif modelNumber == 2
    
    Qp = 1;
    Qu = 0;
    Qs = 0.1;
    Qv = 0;
    Qa = 0.01;
    
    accelerationTerm(1) = Qa*((v(2) - v(1))/(t(2) - t(1)))^.2;
    for i = 1:length(v)-1
        accelerationTerm(i+1) = Qa*((v(i+1) - v(i))/(t(i+1) - t(i)))^.2;
    end
    
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu*(u(1,:)-u(2,:)).^2 + Qs * (v - xRef(4,:)).^2 +...
        Qv * (psiDot - xRef(5,:)).^2 + accelerationTerm);
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*RR/2 + Qv*(psiDot - xRef(5,:))*RR/(2*b); ...
        Qu*(-1)*(u(1,:)-u(2,:)) + Qs*(v - xRef(4,:))*RL/2 - Qv*(psiDot - xRef(5,:))*RL/(2*b)];
    
end