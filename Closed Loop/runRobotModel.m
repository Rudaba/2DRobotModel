global refTraj N n m y0 t0 Hp x Nu Nx M Nui intdt
global  MPCmodelNumber R b t_sort
%*****Define Simulation Parameters***
MPCmodelNumber  = 2;
plotResults     = 0;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
Nx  = 2; % Number of state integration intervals for DS
Nu  = 2; % Number of discrete control points for DS
M   = 2;   % Number of intervals for MS
Nui = 10;   % Number of control points per interval for MS
N   = 50; % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
updateRate  = 0.1;
t0          = 0;
Hp          = 5;
tf          = updateRate;
intdt       = 0.001;
simTime     = 50;

%****Define Robot Parameters*****
%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTraj_circ;

%*****Define Constraints*****
constraints         = 0;
constraintValues    = [-inf,inf];

%*****Define Initial Conditions*****
y0 = [-10;7;0;0]; %This is an offset from the path
y0 = [5;0;0];     %This starts on the path

%*****Intialise model*****
if MPCmodelNumber == 1
    
    % Pseudospectral
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m);
    
elseif MPCmodelNumber == 2
    
    % Pseudospectral
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,n,m,y0,constraintValues,refTraj);
    
end

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level', -1);
snseti('Derivative option', 2);
snseti('Major iterations',1000);

for i = 1:simTime/updateRate
    
    x                       = runMPC(MPCmodelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
    [y0,tReal,yReal, uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,MPCmodelNumber,refTraj);
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

save('testing', 'tout','yout','uout','yrefout');
