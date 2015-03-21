global refTraj N n m xNav t0 tf Hp x Nx M

%*****Define Simulation Parameters***
modelNumber = 1;

%*****Define Model Parameters*****
n   = 3; %Number of states
m   = 2; %Number of controls
M   = 5; %this is for MS only;
Nx  = 4; %this is for MS only;

%*****Define Variable Parameters*****
t0  = 0;
tf  = 30;
Hp  = tf - t0;
N   = 50;

%*****Define Reference Trajectory*****
[refTraj] = [[0:50]',[0:50]',5*ones(51,1),0*ones(51,1)];

%*****Intialise model*****
if modelNumber == 1
    % Direct Shooting
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(N,n,m,refTraj);
    
elseif modelNumber == 2
    % Multiple Shooting
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nx,n,m,refTraj,Hp,M,t0);
    
elseif modelNumber == 3
    % Collocation
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseCollocation(N,n,m,refTraj);
    
elseif modelNumber == 4
    % Pseudospectral
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialisePseudospectral(N,n,m,refTraj);
    
elseif modelNumber == 5
    % Linear MPC
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,refTraj);
end

%*****Store Data*****
storeData(1).time       = t0;
storeData(1).stateVec   = xNav;
% storeData(1).u          = [0;0];

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level     ', -1);
snseti('Derivative option', 0); % let SNOPT figure out jacobian
snseti('Major iterations',1000);

runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar);
