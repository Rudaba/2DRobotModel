function [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nx,n,m,refTraj,Hp,M,t0)

%*****Define Initial Conditions*****
xNav = refTraj(1,2:4)'; %This is initial nav robot state [x;y;psi]

dti = Hp / (M);
t0j = [t0:dti:t0+Hp-dti];
tfj = [t0+dti:dti:t0+Hp];

ICstart =  interp1(refTraj(:,1),refTraj(:,2:end),t0j)';
ICend =  interp1(refTraj(:,1),refTraj(:,2:end),tfj)';

for j = 1:M
    %This fills in the continuity  for the begningng and the end of each
    %interval
    xlow((j-1)*n+1:j*n,1)       = -inf;
    xlow(M*n+((j-1)*n+1:j*n),1) = -inf;
    xupp((j-1)*n+1:j*n,1)       = inf;
    xupp(M*n+((j-1)*n+1:j*n),1) = inf;
    x((j-1)*n+1:j*n,1)          = ICstart(1:n,j);
    x(M*n+((j-1)*n+1:j*n),1)    = ICend(1:n,j);
end

count = 0;
for j = 1:M
    for k = 1:m
        xlow(2*M*n+(j-1)*(Nx+1)+count+((k-1)*(Nx+1)+1:k*(Nx+1)),1) = -inf;
        xupp(2*M*n+(j-1)*(Nx+1)+count+((k-1)*(Nx+1)+1:k*(Nx+1)),1) = inf;
        x(2*M*n+(j-1)*(Nx+1)+count+((k-1)*(Nx+1)+1:k*(Nx+1)),1)    = 0;
    end
    count = j*(Nx+1);
end

neF=1+n*M*2;
Jac = ones(neF,2*n*M + (Nx+1)*M*m);
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
