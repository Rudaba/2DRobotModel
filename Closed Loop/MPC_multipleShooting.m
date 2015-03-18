function [uMPC_OUT,tMPC_OUT] = MPC_multipleShooting(xlow,xupp,Flow,Fupp,iGfun,jGvar)

global x m n M t0 Hp Nx

A = [];
iAfun = [];
jAvar = [];

[x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunMultipleShooting', A, iAfun, jAvar, iGfun, jGvar);

% Extract states and controls from x vector
for j = 1:M
    ystart{j}(:, 1) = x((j-1)*n+1:j*n);
    yend{j}(:, 1) = x(M*n+((j-1)*n+1:j*n));
end

initialTime = t0;
segementLength = Hp / M;
for j = 1:M
    tMPC{j} = initialTime:segementLength/Nx:initialTime+segementLength;
    for k = 1:m
        uMPC{j}(:,k) = x(2*M*n+(j-1)*(Nx+1)+((k-1)*(Nx+1)+1:k*(Nx+1)));
    end
    initialTime = initialTime + segementLength; 
end

uMPC_OUT = uMPC{1};
tMPC_OUT = tMPC{1};

% u = uMPC{1}(1,:)';
