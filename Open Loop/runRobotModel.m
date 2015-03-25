global refTraj N n m xNav t0 tf Hp x Nu Nx M Nui

%*****Define Simulation Parameters***
modelNumber = 3;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
Nx  = 10;  % Number of state integration intervals for DS
Nu  = 5;  % Number of discrete control points for DS
M   = 5;   % Number of intervals for MS
Nui = 2;  % Number of control points per interval for MS
N   = 20;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
t0  = 0;
tf  = 10;
Hp  = tf - t0;


%*****Define Reference Trajectory*****
[refTraj] = [[0:50]',[0:50]',5*ones(51,1),0*ones(51,1)];

%*****Intialise model*****
if modelNumber == 1
    % Direct Shooting
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(Nu,n,m,refTraj);
    
elseif modelNumber == 2
    % Multiple Shooting
    [xNav,x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nui,n,m,refTraj,Hp,M,t0);
    
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
