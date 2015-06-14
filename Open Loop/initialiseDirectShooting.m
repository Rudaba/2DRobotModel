function [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(y0,Nu,n,m)

for k = 1:m %Intialise MPC vector and define upper and lower constraints
    xlow((k-1)*(Nu+1)+1:n+k*(Nu+1),1) = -0.4;
    xupp((k-1)*(Nu+1)+1:n+k*(Nu+1),1) =  0.4;
    
    if k == 1
        x((k-1)*(Nu+1)+1:n+k*(Nu+1),1) = 0;
    elseif k == 2
        x((k-1)*(Nu+1)+1:n+k*(Nu+1),1) =  0;
    end
    
end

neF=1+n;
Jac = ones(neF,(m)*(Nu+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
