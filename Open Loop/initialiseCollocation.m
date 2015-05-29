function [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseCollocation(refTraj,N,n,m,y0)

for j = 1:n
    
    if j == 1
        const = refTraj(1,2);
    elseif j == 2
        const = refTraj(1,3);
    else
        const = refTraj(1,4);
    end
    
    xlow((j-1)*(N+1)+1:j*(N+1),1) = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1) = inf;
    x((j-1)*(N+1)+1:j*(N+1),1) = const;
    
end

for k = 1:m
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = -inf;
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = inf;
    
    if k == 1
        const = refTraj(1,7);
    elseif k == 2
        const = refTraj(1,8);
    end
        
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = const;
end

neF = 1 + n*(N+1) + 2*n; 

Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
