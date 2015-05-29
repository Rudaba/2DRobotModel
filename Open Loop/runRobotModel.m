global refTraj N n m y0 t0 Hp x Nu Nx M Nui intdt
global  MPCmodelNumber R b
%*****Define Simulation Parameters***
MPCmodelNumber  = 2;
plotResults     = 1;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
Nx  = 20;  % Number of state integration intervals for DS
Nu  = 10;  % Number of discrete control points for DS
M   = 5;   % Number of intervals for MS
Nui = 20;   % Number of control points per interval for MS
N   = 100;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
t0    = 0;
Hp    = 5;
intdt = 0.001; 

%****Define Robot Parameters*****
%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTrajectory; 

%*****Define Initial Conditions*****
y0 = [-0.5;5+1;0]; %This is initial nav robot state [x;y;psi]
% y0 = [0;5.1;0]; %This is initial nav robot state [x;y;psi]
% y0 = refTraj(1,2:4)';

%*****Intialise model*****
if MPCmodelNumber == 1
    % Direct Shooting
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(refTraj,Nu,n,m);
    
elseif MPCmodelNumber == 2
    % Multiple Shooting
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nui,n,m,y0,M);
    
elseif MPCmodelNumber == 3
    % Collocation
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseCollocation(refTraj,N,n,m,y0);
    
elseif MPCmodelNumber == 4
    % Pseudospectral
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialisePseudospectral(refTraj,N,n,m,y0);

elseif MPCmodelNumber == 5
    % Pseudospectral
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m);
end

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level', -1);
snseti('Derivative option', 2);
snseti('Major iterations',1000);

runMPC(MPCmodelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar,'testing',plotResults);
