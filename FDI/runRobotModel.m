global refTraj N n m y0 t0 Hp x intdt t_sort modelNumber
global b X_EKF
%*****Define Simulation Parameters***
modelNumber     = 2;
plantFileName   = 'plantData';
EKFFileName     = 'EKFData';
count           = 1;

%*****Define Model Parameters*****
n   = 3;   % Number of states
m   = 2;   % Number of controls states
N   = 50;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
MPCupdateRate   = 0.1;
t0              = 0;
Hp              = 5;
intdt           = 0.001;
simTime         = 50;
EKFUpdateRate   = 0.01;
tf              = MPCupdateRate;

%*****Define Initial Conditions*****
%This is initial nav robot state [x;y;psi]
y0          = [-10;7;0]; %This is an offset from the path
y0          = [5;0;0];    %This starts on the path
u           = [0;0];
measurement = y0;
% measurement = measurement + 0.01*rand(3,1);

%****Define Robot Parameters*****
%Robot Constants
RR      = 2; %Radius of right tyre
RL      = 2; %Radius of left tyre
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTraj_circ;

%*****Define Constraints*****
constraints         = 1;
% constraintValues    = [-inf,inf];
constraintValues    = [-5,5];

%*****Intialisation*****
if modelNumber == 1
    
    % MPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,constraintValues);
    
elseif modelNumber == 2
    
    % NMPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,n,m,y0,constraintValues);
    
end

[X_EKF, P, Q, R_Noise] = initialiseEKF(y0,RR, RL);

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level', -1);
snseti('Derivative option', 0); % let SNOPT figure out jacobian
% snseti('Derivative option', 2); % provide Jacobians
snseti('Major iterations',1000);

%*******************Start Simulation***************************************
%At time t0 = 0
[X_EKF,P,innovations]   = EKFUpdate(X_EKF,P,u,EKFUpdateRate,measurement,Q,R_Noise);

x                       = runMPC(modelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
[y0,tReal,yReal, uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,modelNumber,refTraj);
yrefstore               = interp1(refTraj(:,1),refTraj(:,2:4),tReal)';

plantData(count).y      = yReal';
plantData(count).t      = tReal';
plantData(count).u      = uReal';
plantData(count).yref   = yrefstore;

EKFData(1).t            = t0;
EKFData(1).X            = X_EKF;
EKFData(1).P            = diag(P);
EKFData(1).innovations  = innovations;

count                   = count + 1;
tf                      = tf + MPCupdateRate;
t0                      = t0 + EKFUpdateRate;

u                       = interp1(tReal(:,1),uReal(:,1:2),t0,'pchip')';
u                       = u + 0.01*rand(2,1);

measurement             = interp1(tReal(:,1),yReal(:,1:3),t0,'pchip')';
measurement             = measurement + 0.01*rand(3,1);

%Remaining simulation time
for i = 1:simTime/EKFUpdateRate
    
    [X_EKF,P] = EKFUpdate(X_EKF,P,u,EKFUpdateRate,measurement,Q,R_Noise);
    
    EKFData(i+1).t              = t0;
    EKFData(i+1).X              = X_EKF;
    EKFData(i+1).P              = diag(P);
    EKFData(i+1).innovations    = innovations;
    
    if mod(i,MPCupdateRate/EKFUpdateRate) == 0
        
        x                       = runMPC(modelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
        [y0,tReal,yReal, uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,modelNumber,refTraj);
        yrefstore               = interp1(refTraj(:,1),refTraj(:,2:4),tReal)';
        
        plantData(count).y     = yReal';
        plantData(count).t     = tReal';
        plantData(count).u     = uReal';
        plantData(count).yref  = yrefstore;
        
        count = count + 1;
        
        tf = tf + MPCupdateRate;
        
    end
    
    t0 = t0 + EKFUpdateRate;
    
    u            = interp1(tReal(:,1),uReal(:,1:2),t0,'pchip')';
    u            = u + 0.01*rand(2,1);
    
    measurement  = interp1(tReal(:,1),yReal(:,1:3),t0,'pchip')';
    measurement  = measurement + 0.01*rand(3,1);
    
end

plant_tout    = [plantData.t];
plant_yout    = [plantData.y];
plant_uout    = [plantData.u];
plant_yrefout = [plantData.yref];

save(plantFileName, 'plant_tout','plant_yout','plant_uout','plant_yrefout');

EKF_tout        = [EKFData.t];
EKF_Xout        = [EKFData.X];
EKF_Pout        = [EKFData.P];
EKF_Innovations = [EKFData.innovations];

save(EKFFileName, 'EKF_tout','EKF_Xout','EKF_Pout','EKF_Innovations');