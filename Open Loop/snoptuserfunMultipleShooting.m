function [F,Jac,tout,yout,uout] = snoptuserfunMultipleShooting(x)

global M t0 tf y0 n m refTraj Nui

numconstr = 1 + 2*n*M;

F  = zeros(numconstr,1);

% Extract states and controls from x vector
for j = 1:M
    ystart{j}(:, 1) = x((j-1)*n+1:j*n);
    yend{j}(:, 1) = x(M*n+((j-1)*n+1:j*n));
end

count = 0;
for j = 1:M
    for k = 1:m
        u{j}(:,k) = x(2*M*n+(j-1)*(Nui+1)+count+((k-1)*(Nui+1)+1:k*(Nui+1)));
    end
    count = j*(Nui+1);
end

% Integrate the state equations
dti = (tf - t0) / (M);
dtu = dti / (Nui);
t0j = [t0:dti:tf-dti];
tfj = [t0+dti:dti:tf];

for j = 1:M
    tu{j} = [t0j(j):dtu:tfj(j)];
end

cost = 0;

% if nargout > 3
%     for j = 1:M
%         yout{j}(:,1) = ystart{j};
%     end
% end

uout = u;

for j = 1:M
    y = ystart{j};
    for i = 1:Nui
        xRef        = interp1(refTraj(:,1),refTraj(:,2:end),tu{j}(i))';
        [yDots]     = stateEquations(y', u{j}(i,:),tu{j}(i));
        [Mayer, Integral]  = integralCost(y,u{j}(i,:)', xRef);
        
        y = y + yDots * dtu;
        cost = cost + Integral * dtu;
        
        
        
        %         if nargout > 3
        %             yout{j}(:,end+1) = y;
        %         end
        
    end
    yfinal{j} = y;
end

% if nargout > 2
%     tout = tu;
% end

F(1) = cost;

for j = 1:M
    if j == 1
        F(1 + ((j-1)*n+1:j*n)) = ystart{j} - y0;
    else
        F(1 + ((j-1)*n+1:j*n)) = ystart{j} - yend{j-1};
    end
    
    F(1 + n*M + ((j-1)*n+1:j*n)) = yend{j} - yfinal{j};
end

Jac= [];