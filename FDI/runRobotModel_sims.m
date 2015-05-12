function runRobotModel_sims(constraints, constraintValues, model, initialConditions, fileString)

global refTraj N n m y0 t0 Hp x intdt t_sort modelNumber
global R b

%*****Define Simulation Parameters***
modelNumber = model;
fileName = fileString;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
N   = 50;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
updateRate  = 0.1;
t0          = 0;
Hp          = 5;
tf          = updateRate;
intdt       = 0.01;
simTime     = 50;

%*****Define Initial Conditions*****
%This is initial nav robot state [x;y;psi]
y0 = initialConditions;

%****Define Robot Parameters*****
%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTraj_circ;

%*****Intialise model*****
if modelNumber == 1
    
    % MPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,constraintValues);
    
elseif modelNumber == 2
    
    % NMPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,n,m,y0,constraintValues);
    
end

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen off;

snseti('Verify level', -1);
snseti('Derivative option', 2); % let SNOPT figure out jacobian
snseti('Major iterations',1000);

for i = 1:simTime/updateRate
    x                       = runMPC(modelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
    [y0,tReal,yReal, uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,modelNumber,refTraj);
    yrefstore               = interp1(refTraj(:,1),refTraj(:,2:4),tReal)';
    
    stored(i).y = yReal';
    stored(i).t = tReal';
    stored(i).u = uReal';
    stored(i).yref  = yrefstore;
    
    t0 = tf;
    tf = t0 + updateRate;
    
end

tout    = [stored.t];
yout    = [stored.y];
uout    = [stored.u];
yrefout = [stored.yref];

save(fileName, 'tout','yout','uout','yrefout');
