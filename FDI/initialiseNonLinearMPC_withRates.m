function [x,n,m,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC_withRates(N,y0,constraintValues)
global D_sort w t_sort

%*****Define Number of states and controls*****
n   = 5;   % Number of states
m   = 2;   % Number of controls states

%The number of states are 3 (x, y and psi) the number of control states are
%2 (omegaR and omegaL) hence the number or rates on the controls is 2

%*****Calculate differentiation matrix, nodes, and quadrature weights*****
[t_sort,w]  = LegendreNodesAndWeights(N);
D_sort      = ComputeDifferentiationMatrix(N,t_sort);


%*****Define Initial Conditions*****
for j = 1:(n-m) %For states [x, y, psi]
    
    if j == 1
        const = y0(1,1);
    elseif j == 2
        const = y0(2,1);
    elseif j == 3
        const = y0(3,1);
    end
    
    xlow((j-1)*(N+1)+1:j*(N+1),1) = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1) = inf;
    x((j-1)*(N+1)+1:j*(N+1),1) = const;
    
end

for k = 1:m %For controls 
    xlow((n-m)*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(1);
    xupp((n-m)*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(2);
    x((n-m)*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0;
end

for k = 1:m %For control rates
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = -1000*pi/180;
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 1000*pi/180;
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0;
end


neF = 1 + n*(N+1) + n; %(1 for cost n*N for eq constraints and n for BC's)
Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
