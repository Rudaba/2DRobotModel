function [A,B,V,PSI,PSI_DOT] = Statespace(xRef,omegaR,omegaL)

%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%Extract Data
x       = xRef(1);
y       = xRef(2);
psi     = xRef(3);
V       = R * (omegaR + omegaL)/2; 
PSI_DOT = R * (omegaR - omegaL)/(2*b);
PSI     = psi;

% omegaR = u(1,1);
% omegaL = u(1,1);

A = [0 0 -V*sin(psi); ...
    0 0 V*cos(psi);...
    0 0 0];

B = [ R/2*cos(psi) R/2*cos(psi);...
    R/2*sin(psi) R/2*sin(psi);...
    R/(2*b)      -R/(2*b)];