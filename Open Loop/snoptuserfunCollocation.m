function [F,Jac,tout,yout,uout] = snoptuserfunCollocation(x)

global N t0 Hp y0 n m refTraj

numconstr   = 1 + n*(N+1) + 2*n;

F  = zeros(numconstr,1);

% Extract states and controls from x vector
for j = 1:n
  y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
  u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end


% Integrate the state equations
tf   = t0 + Hp;
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

F(1) = cost;

% initial conditions
F(1+n.*(N+1)+1) = y(1,1)- y0(1);
F(1+n.*(N+1)+2) = y(1,2)- y0(2);
F(1+n.*(N+1)+3) = y(1,3)- y0(3);

%Terminal constraints
F(1+n.*(N+1)+4) = y(N+1,1) - xRef(1,end);
F(1+n.*(N+1)+5) = y(N+1,2) - xRef(2,end);
F(1+n.*(N+1)+6) = y(N+1,3) - xRef(3,end);


Jac= [];