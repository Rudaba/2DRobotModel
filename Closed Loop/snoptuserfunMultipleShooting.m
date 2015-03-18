function [F,Jac,tout,yout,uout] = snoptuserfunMultipleShooting(x)

global M t0 Hp xNav n m refTraj Nx

numconstr = 1 + 2*n*M;

F  = zeros(numconstr,1);

% Extract states and controls from x vector
for j = 1:M
    ystart{j}(:, 1) = x((j-1)*n+1:j*n);
    yend{j}(:, 1) = x(M*n+((j-1)*n+1:j*n));
end
for j = 1:M
    for k = 1:m
        u{j}(:,k) = x(2*M*n+(j-1)*(Nx+1)+((k-1)*(Nx+1)+1:k*(Nx+1)));
    end
end

% Integrate the state equations
dti = Hp / (M);
dtu = dti / (Nx);
t0j = t0 + [0:dti:Hp-dti];
tfj = t0 + [dti:dti:Hp];
for j = 1:M
    tu{j} = [t0j(j):dtu:tfj(j)];
end

cost = 0;

% if nargout > 3
%   for j = 1:M
%     yout{j}(:,1) = ystart{j};
%   end
% end

for j = 1:M
    stateVec = ystart{j};
    for i = 1:Nx
        xRef        = interp1(refTraj(:,1),refTraj(:,2:end),tu{j}(i))';
        stateVec    = stateEquations_MS(stateVec, u{j}(i,:)', dtu);
        integrand   = cost_MS(stateVec,u{j}(i,:)',xRef);
        
        cost        = cost + integrand * dtu;
        
        %     if nargout > 3
        %       yout{j}(:,end+1) = y;
        %     end
        
    end
    yfinal{j} = stateVec;
end

% if nargout > 2
%     tout = tu;
% end

F(1) = cost;

y0 = xNav;

for j = 1:M
    if j == 1
        F(1 + ((j-1)*n+1:j*n)) = ystart{j} - y0;
    else
        F(1 + ((j-1)*n+1:j*n)) = ystart{j} - yend{j-1};
    end
    
    F(1 + n*M + ((j-1)*n+1:j*n)) = yend{j} - yfinal{j};
end

Jac= [];