function runRobotModel_sims(model,initTime,finalTime,statePointsDS,controlPoints,sections,statePointsMS,nodePoints,fileName)
                            
global refTraj N n m y0 t0 tf Hp x Nu Nx M Nui intdt
global R b

%*****Define Simulation Parameters***
modelNumber = model;
plotResults = 0;

%*****Define Model Parameters*****
n   = 3;                % Number of states
m   = 2;                % Number of controls states
Nx  = statePointsDS;    % Number of state integration intervals for DS
Nu  = controlPoints;    % Number of discrete control points for DS
M   = sections;         % Number of intervals for MS
Nui = statePointsMS;    % Number of control points per interval for MS
N   = nodePoints;       % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
t0      = initTime;
tf      = finalTime;
Hp      = tf - t0;
intdt   = 0.01; %Integration time step

%*****Define Initial Conditions*****
y0 = [-0.5;5+1;0]; %This is initial nav robot state [x;y;psi]

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
end

%*****Set up optimisation*****
%SNOPT parameters
summaryName = strcat(fileName,'_Summary');
snsummary(summaryName);
snscreen off;

snseti('Verify level', -1);
snseti('Derivative option', 0); % let SNOPT figure out jacobian
snseti('Major iterations',1000);

runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar,fileName,plotResults);
