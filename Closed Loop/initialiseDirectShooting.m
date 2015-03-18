function [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(N,n,m,refTraj)

%*****Define Initial Conditions*****
xNav = refTraj(1,2:4)';; %This is initial nav robot state [x;y;psi]

for k = 1:m %Intialise MPC vector and define upper and lower constraints
    xlow((k-1)*(N+1)+1:n+k*(N+1),1) = -inf;
    xupp((k-1)*(N+1)+1:n+k*(N+1),1) =  inf;
    x((k-1)*(N+1)+1:n+k*(N+1),1) =  0;
end

neF=1+n;
Jac = ones(neF,(m)*(N+1));
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
