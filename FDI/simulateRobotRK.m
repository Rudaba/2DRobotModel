function [y0,T,Y,U] = simulateRobotRK(y0,tref,uref,dt,initTime,finalTime)
%Extract Data
psi     = y0(3);
y       = y0(2);
x       = y0(1);

% omegaR  = u(1);
% omegaL  = u(2);

%v       = R*(omegaR+omegaL)/2;

[T,Y] = ode45(@(t,y) diffEqns(t,y,tref,uref),[initTime:dt:finalTime],[x y psi]);
% [T,Y] = ode45(@(t,y) diffEqns(t,y,tref,uref),[initTime finalTime],[x y psi]);

omegaR = interp1(tref, uref(:,1), T, 'pchip');
omegaL = interp1(tref, uref(:,2), T, 'pchip');
U      = [omegaR,omegaL];

% %Solve DE's
% psiDot  = R*(omegaR-omegaL)/(2*b);
% psi     = psi + psiDot * dt;
% yDot    = v*sin(psi);
% y       = y + yDot * dt;
% xDot    = v*cos(psi);
% x       = x + xDot * dt;

y0    = Y(end,:)';%[x;y;psi];
% y0    = [y0; U(end,:)'];

function dy = diffEqns(t,y,tref,uref)
global b faultScenarioCase faultOccurrenceFlag

omegaR = interp1(tref, uref(:,1), t, 'pchip');
omegaL = interp1(tref, uref(:,2), t, 'pchip');

RR     = 2;
RL     = 2;

if faultOccurrenceFlag == 1
    switch faultScenarioCase
        case 1
            if t >= 20
                RR     = 2;
                RL     = 0.5*2;
            end
        case 2
            if t >= 20
                RR     = 0.5*2;
                RL     = 2;
            end
        case 3
            if t >= 15
                RR     = 0.5*2;
            end
            if t >= 30
                RL     = 2;
            end
        otherwise
            disp('Unknown fault Scenario')
    end
end

v       = RR*omegaR/2 + RL*omegaL/2;

dy = zeros(3,1);    % a column vector
dy(1) = v * cos(y(3));
dy(2) = v * sin(y(3));
dy(3) = RR*omegaR/(2*b) - RL*omegaL/(2*b);