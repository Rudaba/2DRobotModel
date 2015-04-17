function [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nui,n,m,y0,M)

% dti = Hp / (M);
% t0j = [t0:dti:t0+Hp-dti];
% tfj = [t0+dti:dti:t0+Hp];
% 
% ICstart =  interp1(refTraj(:,1),refTraj(:,2:end),t0j)';
% ICend =  interp1(refTraj(:,1),refTraj(:,2:end),tfj)';

for j = 1:M
  xlow((j-1)*n+1:j*n,1)         = -inf;
  xlow(M*n+((j-1)*n+1:j*n),1)   = -inf;
  xupp((j-1)*n+1:j*n,1)         = inf;
  xupp(M*n+((j-1)*n+1:j*n),1)   = inf;
  x((j-1)*n+1:j*n,1)            = 0;
  x(M*n+((j-1)*n+1:j*n),1)      = 0;
end

count = 0;
for j = 1:M
    for k = 1:m
        xlow(2*M*n+(j-1)*(Nui+1)+count+((k-1)*(Nui+1)+1:k*(Nui+1)),1) = -0.6;
        xupp(2*M*n+(j-1)*(Nui+1)+count+((k-1)*(Nui+1)+1:k*(Nui+1)),1) = 0.6;
        x(2*M*n+(j-1)*(Nui+1)+count+((k-1)*(Nui+1)+1:k*(Nui+1)),1)    = 0;
    end
    count = j*(Nui+1);
end

neF=1+n*M*2;
Jac = ones(neF,2*n*M + (Nui+1)*M*m);
[iGfun,jGvar,G]=find(Jac);

Flow = zeros(neF,1);
Fupp = zeros(neF,1);
Flow(1)  = -Inf;    Fupp(1) =  Inf;    % Leave Objective Unbounded
