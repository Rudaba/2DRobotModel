function [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,refTraj)
global D_sort w t_sort

%*****Calculate differentiation matrix, nodes, and quadrature weights*****
[t_sort,w]  = LegendreNodesAndWeights(N);
D_sort      = ComputeDifferentiationMatrix(N,t_sort);


neF = 1 + n*(N+1) + 2*n; %(1 for cost n*N for eq constraints and n for intial and terminal constraints)
Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded

%*****Define Initial Conditions*****
xNav = refTraj(1,2:4)';

for j = 1:n
    xlow((j-1)*(N+1)+1:j*(N+1),1)    = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1)    = inf;
    x((j-1)*(N+1)+1:j*(N+1),1)       = 0;
end

for k = 1:m
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = -inf;
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = inf;
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0;
end
