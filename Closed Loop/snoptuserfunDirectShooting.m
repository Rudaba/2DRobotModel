function [F,Jac,tout,yout,uout] = snoptuserfunDirectShooting(x)

global N t0 Hp xNav n m refTraj;

numconstr = 1 + n;

F  = zeros(numconstr,1);
u  = zeros(N+1,m);

for k = 1:m
  u(:,k) = x((k-1)*(N+1)+1:k*(N+1));
end

% Integrate the state equations
dtx = Hp / (N);
tx = [t0:dtx:t0+Hp];
dtu = Hp / (N);
tu = [t0:dtu:t0+Hp];

stateVec    = xNav;
cost        = 0;

% if nargout > 2
%   tout = tx;
% end

% if nargout > 3
%   yout(:,1) = y0;
%   uout(:,1) = u(1,:);
% end

for i = 1:N
  % zero order hold for u
    tuIndex = find(tu<=tx(i),1,'last');
    uapplied = u(tuIndex,:)';
    xRef    = interp1(refTraj(:,1),refTraj(:,2:end),tx(i))';
  
  %linear interp
%   uapplied = interp1(tu, u, tx(i), 'linear');
  
  stateVec          = stateEquations_DS(stateVec, uapplied, dtx);
  integrand         = cost_DS(stateVec, uapplied, xRef);
  
  %y = y + ydot * dtx;
  cost = cost + integrand * dtx;
  
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