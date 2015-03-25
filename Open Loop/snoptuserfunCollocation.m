function [F,Jac,tout,yout,uout] = snoptuserfunCollocation(x)

global N t0 tf xNav n m refTraj

numconstr = 1 + n*(N+1);

F  = zeros(numconstr,1);

% Extract states and controls from x vector
for j = 1:n
  y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
  u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end


% Integrate the state equations
dt   = (tf - t0) / (N);
t    = [t0:dt:tf];
cost = 0;

% if nargout > 3
%   yout = y;
%   uout = u;
% end

yDots                   = stateEquations(y, u, t);
xRef                    = interp1(refTraj(:,1),refTraj(:,2:end),t)';
[Mayer, Integral]       = integralCost(y',u',xRef);
cost                    = sum(Integral) * dt;%sum(integrand) * dt;

for i = 1:N
  
  F(1+((i-1)*n+1:i*n)) = y(i,:)' - y(i+1,:)' + (yDots(:,i) + yDots(:,i+1)) * dt * 0.5;
  
end

% if nargout > 2
%   tout = t;
% end

y0 = xNav;

F(1) = cost;
% BC's
F(1 + n*N + (1:n)) = y(1,:) - y0';

Jac= [];