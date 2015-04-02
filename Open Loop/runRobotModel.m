global refTraj N n m y0 t0 tf Hp x Nu Nx M Nui
global R b
%*****Define Simulation Parameters***
modelNumber = 4;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
Nx  = 2; % Number of state integration intervals for DS
Nu  = 4; % Number of discrete control points for DS
M   = 5;   % Number of intervals for MS
Nui = 2;  % Number of control points per interval for MS
N   = 10;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
t0  = 0;
tf  = 10;
Hp  = tf - t0;

%*****Define Initial Conditions*****
y0 = [-0.5;5+1;0]; %This is initial nav robot state [x;y;psi]
% y0 = [0;5;0];

%****Define Robot Parameters*****
%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = [[0:50]',[0:50]',5*ones(51,1),0*ones(51,1),ones(51,1)]; % PW: ADDED SPEED

%*****Intialise model*****
if modelNumber == 1
    % Direct Shooting
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseDirectShooting(Nu,n,m);
    
elseif modelNumber == 2
    % Multiple Shooting
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseMultipleShooting(Nui,n,m,y0,M);
    
elseif modelNumber == 3
    % Collocation
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseCollocation(N,n,m,y0);
    
elseif modelNumber == 4
    % Pseudospectral
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialisePseudospectral(N,n,m,y0);
    
elseif modelNumber == 5
    % Linear MPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,refTraj);
end

%*****Store Data*****
storeData(1).time       = t0;
storeData(1).stateVec   = y0;
% storeData(1).u          = [0;0];

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level     ', -1);
snseti('Derivative option', 0); % let SNOPT figure out jacobian
snseti('Major iterations',1000);

runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar);
