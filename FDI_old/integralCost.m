function [Mayer, Integral, dC_dx] = integralCost(x,u,v,psiDot,xRef, RR, RL)

global b MPCmodelNumber N

Mayer = 0;

if MPCmodelNumber == 1
    
    %     Qp = 5;
    %     Qv = 10;
    %     Qs = 10;
    %
    %     Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qv * (v).^2 + Qs * (psiDot).^2);
    %
    %     dC_dx = [Qp*x(1,:); ...
    %         Qp*x(2,:); ...
    %         Qp*x(3,:); ...
    %         Qv*v*RR/2 + Qs*psiDot*RR/(2*b); ...
    %         Qv*v*RL/2 - Qs*psiDot*RL/(2*b)];
    %
    %         Qp = 5;
    %         Qu = 10;
    %         Qv = 10;
    %         Qs = 10;
    %
    %         Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2 + Qv * (v).^2 + Qs * (psiDot).^2);
    %
    %         dC_dx = [Qp*x(1,:); ...
    %             Qp*x(2,:); ...
    %             Qp*x(3,:); ...
    %             Qu*u(1,:) + Qv*v*RR/2 + Qs*psiDot*RR/(2*b); ...
    %             Qu*u(2,:) + Qv*v*RL/2 - Qs*psiDot*RL/(2*b)];
    %
    
    Qp = 100;
    Qu = 0.1;
    
    Integral = 0.5*(Qp*x(1,:).^2 + Qp*x(2,:).^2 + Qp*x(3,:).^2 + Qu*u(1,:).^2 + Qu*u(2,:).^2);
    
    dC_dx = [Qp*x(1,:); ...
        Qp*x(2,:); ...
        Qp*x(3,:); ...
        Qu*u(1,:); ...
        Qu*u(2,:)];
    
    
elseif MPCmodelNumber == 2
    
    %     Qp = 5;
    %     Qv = 10;
    %     Qs = 10;
    %
    %     Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qv * (v - xRef(4,:)).^2 + Qs * (psiDot - xRef(5,:)).^2);
    %
    %     dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
    %         Qp*(x(2,:) - xRef(2,:)); ...
    %         Qp*(x(3,:) - xRef(3,:)); ...
    %         Qv*(v - xRef(4,:))*RR/2 + Qs*(psiDot - xRef(5,:))*RR/(2*b); ...
    %         Qv*(v - xRef(4,:))*RL/2 + Qs*(-1)*(psiDot - xRef(5,:))*RL/(2*b)];
    
    Qp = 1;
    Qu = 50;
    
    Integral = 0.5*(Qp*(x(1,:) - xRef(1,:)).^2 + Qp*(x(2,:)-xRef(2,:)).^2 + Qp*(x(3,:)-xRef(3,:)).^2 + Qu * (u(1,:) - xRef(6,:)).^2 + Qu * (u(2,:) - xRef(7,:)).^2);
    
    dC_dx = [Qp*(x(1,:) - xRef(1,:)); ...
        Qp*(x(2,:) - xRef(2,:)); ...
        Qp*(x(3,:) - xRef(3,:)); ...
        Qu*(u(1,:) - xRef(6,:)); ...
        Qu*(u(2,:) - xRef(7,:))];
    
end