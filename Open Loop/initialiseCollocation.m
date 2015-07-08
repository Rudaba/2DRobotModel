function [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseCollocation(N,n,m,y0)

for j = 1:n
    
    if j == 1
        const = y0(1,1);
    elseif j == 2
        const = y0(2,1);
    else
        const = y0(3,1);
    end
    
    xlow((j-1)*(N+1)+1:j*(N+1),1) = -inf;
    xupp((j-1)*(N+1)+1:j*(N+1),1) = inf;
    x((j-1)*(N+1)+1:j*(N+1),1) = const;
    
end

for k = 1:m
    xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = -0.4;
    xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0.4;
    
    x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = 0;
end

neF = 1 + n*(N+1) + 2*n; 

Jac = ones(neF,(n+m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
