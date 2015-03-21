function [xNav,T,Y] = robotNav(xNav,tref,uref,dt,initTime,finalTime)

%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%Extract Data
psi     = xNav(3);
y       = xNav(2);
x       = xNav(1);

% omegaR  = u(1);
% omegaL  = u(2);

%v       = R*(omegaR+omegaL)/2;

[T,Y] = ode45(@(t,y) diffEqns(t,y,tref,uref),[initTime:dt:finalTime],[x y psi]);

% %Solve DE's
% psiDot  = R*(omegaR-omegaL)/(2*b);
% psi     = psi + psiDot * dt; 
% yDot    = v*sin(psi);
% y       = y + yDot * dt;
% xDot    = v*cos(psi);
% x       = x + xDot * dt;

xNav    = Y(end,:)';%[x;y;psi];

function dy = diffEqns(t,y,tref,uref)

omegaR = interp1(tref, uref(:,1), t, 'pchip');
omegaL = interp1(tref, uref(:,2), t, 'pchip');

R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

v       = R*(omegaR+omegaL)/2;

dy = zeros(3,1);    % a column vector
dy(1) = v * cos(y(3));
dy(2) = v * sin(y(3));
dy(3) = R*(omegaR-omegaL)/(2*b);