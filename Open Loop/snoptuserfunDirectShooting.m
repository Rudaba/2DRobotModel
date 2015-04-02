function [F,Jac,tout,yout,uout] = snoptuserfunDirectShooting(x)

global Nu Nx t0 tf n m refTraj y0;

numconstr = 1 + n;

F  = zeros(numconstr,1);
u  = zeros(Nu+1,m);

for k = 1:m
  u(:,k) = x((k-1)*(Nu+1)+1:k*(Nu+1));
end

% Integrate the state equations
dtx = (tf - t0) / (Nx);
tx  = [t0:dtx:tf];
dtu = (tf - t0) / (Nu);
tu  = [t0:dtu:tf];

y     = y0;
cost  = 0;

% if nargout > 2
%   tout = tx;
% end

% if nargout > 3
%   yout(:,1) = xNav;
%   uout(:,1) = u(1,:);
% end

for i = 1:Nx
  % zero order hold for u
    tuIndex     = find(tu<=tx(i),1,'last');
    uapplied    = u(tuIndex,:)';
    xRef        = interp1(refTraj(:,1),refTraj(:,2:end),tx(i))';

  
  %linear interp
%   uapplied = interp1(tu, u, tx(i), 'linear');
  
  [yDots]            = stateEquations(y', uapplied',tx(i));
  [Mayer, Integral]  = integralCost(y, uapplied, xRef);
  
  y          = y + yDots * dtx;
  cost       = cost + Integral * dtx;
  
%   if nargout > 3
%     yout(:,end+1) = y;
%     uout(:,end+1) = uapplied;
%   end
  
end

F(1) = cost;
for k = 1:n
  % no final states
  F(1+k) = 0;%yf(k) - y(k);
end

Jac= [];