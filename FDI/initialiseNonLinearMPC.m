function [x,n,m,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,refTraj,u,constraintValues)
global D_sort w t_sort t0 Hp

%*****Define Number of states and controls*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states

%*****Calculate differentiation matrix, nodes, and quadrature weights*****
[t_sort,w]  = LegendreNodesAndWeights(N);
D_sort      = ComputeDifferentiationMatrix(N,t_sort);

tf          = t0 + Hp;
t           = ((tf-t0)/2*t_sort+(tf+t0)/2);
xRef        = interp1(refTraj(:,1),refTraj(:,2:end),t)';


%*****Define Initial Conditions*****
for j = 1:n
    
    if j == 1
        const = xRef(1,:);%y0(1,1);
    elseif j == 2
        const = xRef(2,:);%y0(2,1);
    elseif j == 3
        const = xRef(3,:);%y0(3,1);
    end
    
    xlow((j-1)*(N+1)+1:j*(N+1),1) = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1) = inf;
    x((j-1)*(N+1)+1:j*(N+1),1) = const;
    
end

for k = 1:m
     if k == 1
         init_u = xRef(6,:);
     else
         init_u = xRef(7,:);
     end
    
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(1);
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(2);
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = init_u;
end

neF = 1 + n*(N+1) + (1*n); %(1 for cost n*N for eq constraints and n for BC's)
Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
