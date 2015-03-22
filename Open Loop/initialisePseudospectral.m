function [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialisePseudospectral(N,n,m,refTraj)
global D_sort w t_sort

%*****Calculate differentiation matrix, nodes, and quadrature weights*****
[t_sort,w]  = LegendreNodesAndWeights(N);
D_sort      = ComputeDifferentiationMatrix(N,t_sort);


%*****Define Initial Conditions*****
xNav = [0;0;0]; %This is initial nav robot state [x;y;psi]

for j = 1:n
    
    if j == 1
        const = xNav(1,1);
    elseif j == 2
        const = xNav(2,1);
    elseif j == 3
        const = xNav(3,1);
    end
    
    xlow((j-1)*(N+1)+1:j*(N+1),1) = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1) = inf;
    x((j-1)*(N+1)+1:j*(N+1),1) = const;
    
end

for k = 1:m
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = -inf;
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = inf;
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0;
end

neF = 1 + n*(N+1) + n; %(1 for cost n*N for eq constraints and n for BC's)
Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
