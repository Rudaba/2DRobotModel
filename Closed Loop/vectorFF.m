function F = vectorFF(x,xRef)

omegaR = x(1);
omegaL = x(2);

%Robot Constants
R   = 2; %Radius of tyres
b   = 1; %Distance between centre of tyres

%Extract states
V           = xRef(4);
psiDot      = xRef(5);

F = [V - (R * (x(1) + x(2)) / 2); 
    psiDot - (R/(2*b) * (x(1) - x(2)))];

% %Robot Constants
% R   = 2; %Radius of tyres
% b   = 1; %Distance between centre of tyres
% 
% %Extract states
% x   = xRef(1);
% y   = xRef(2);
% psi = xRef(3);
% 
% V = R * (omegaR + omegaL) / 2;
% 
% psiDot  = 1;%R/(2*b) * (omegaR - omegaL);
%  
% xDot = V * cos(psi);
% 
% yDot = V * cos(psi);
% 
% F = [xDot; yDot; psiDot];