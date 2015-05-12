function D = ComputeDifferentiationMatrix(N,t,alpha,beta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DIFFERENTIATION MATRIX
% Ref. Williams (2004)

if nargin < 3
    alpha = 0;
end
if nargin < 4
    beta = 0;
end

D = zeros(N+1,N+1);
D(1,1) = (alpha-N*(N+alpha+beta+1))/2/(beta+2);
D(N+1,N+1) = (N*(N+alpha+beta+1)-beta)/(2*(alpha+2));

P0 = jacobf(N,alpha,beta,-1);
PN = jacobf(N,alpha,beta,1);

for j=1:N-1
   jm=j+1;
   Pj = jacobf(N,alpha,beta,t(jm));
   D(1,jm) = -P0/Pj/(beta+1)/(1+t(jm));
   D(N+1,jm) = PN/(alpha+1)/Pj/(1-t(jm));
   D(jm,1) = Pj*(beta+1)/P0/(1+t(jm));
   D(jm,N+1) = -Pj*(alpha+1)/PN/(1-t(jm));
   D(jm,jm) = 0.5*((alpha+beta)*t(jm)+alpha-beta)/(1-t(jm)^2);
   D(1,N+1) = -0.5*(alpha+1)/(beta+1)*P0/PN;
   D(N+1,1) = 0.5*(beta+1)/(alpha+1)*PN/P0;
   for i=1:N-1
      im = i+1;
      Pi = jacobf(N,alpha,beta,t(im));
      if im~=jm
         D(im,jm) = Pi/Pj/(t(im)-t(jm));
      end
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Evaluate Jacobi Polynomial
% Ref. Maple V
% Inputs:   N   -   Degree of polynomial
%           alpha   -   Parameter for determining Jacobi polynomial (>-1)
%           beta    -   Parameter for determining Jacobi polynomial (>-1)
%           x       -   Value of x in interval [-1,1] to evaluate Jacobi polynomial
% Output:   poly    -   Value of polynomial at x

function [poly] = jacobf(N,alpha,beta,x)
y = 0.5*(alpha+beta+2)*x+0.5*(alpha-beta);
yp = 1;
for j=2:N
   ym = y;
   y=(2*j+alpha+beta-1)*(alpha^2-beta^2+(2*j+alpha+beta-2)*(2*j+alpha+beta)*x)/(2*j*(j+alpha+beta)*(2*j+alpha+beta-2))*y ...
      -2*(j+alpha-1)*(j+beta-1)*(2*j+alpha+beta)/(2*j*(j+alpha+beta)*(2*j+alpha+beta-2))*yp;
   yp = ym;
end
poly = y;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%