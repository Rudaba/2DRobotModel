global refTraj N n m y0 t0 Hp x intdt t_sort modelNumber
global R b
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
updateRate      = 0.1;
t0              = 0;
Hp              = 5;
intdt           = 0.01;
simTime         = 10;
EKFUpdateRate   = 0.01;
tf              = updateRate;

%*****Define Initial Conditions*****
%This is initial nav robot state [x;y;psi]
y0          = [-10;7;0;0]; %This is an offset from the path
y0          = [5;0;0];    %This starts on the path
u           = [0;0];
measurement = y0;
measurement = measurement + 0.01*rand(3,1);

%****Define Robot Parameters*****
%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTraj_circ;

%*****Define Constraints*****
constraints         = 0;
constraintValues    = [-inf,inf];

%*****Intialisation*****
if modelNumber == 1
    
    % MPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,n,m,constraintValues);
    
elseif modelNumber == 2
    
    % NMPC
    [x,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,n,m,y0,constraintValues);
    
end

[X_EKF, P, Q, R_Noise] = initialiseEKF(y0,R);

%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level', -1);
snseti('Derivative option', 2); % let SNOPT figure out jacobian
snseti('Major iterations',1000);

%*******************Start Simulation***************************************
%At time t0 = 0
[X_EKF,P]               = EKFUpdate(X_EKF,P,u,EKFUpdateRate,measurement,Q,R_Noise);

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

count                   = count + 1;
tf                      = tf + updateRate;
t0                      = t0 + EKFUpdateRate;

u                       = interp1(tReal(:,1),uReal(:,1:2),t0)';
measurement             = interp1(tReal(:,1),yReal(:,1:3),t0)';

measurement             = measurement + 0.01*rand(3,1);

%Remaining simulation time
for i = 1:simTime/EKFUpdateRate
    
    [X_EKF,P] = EKFUpdate(X_EKF,P,u,EKFUpdateRate,measurement,Q,R_Noise);
    
    EKFData(i+1).t        = t0;
    EKFData(i+1).X        = X_EKF;
    EKFData(i+1).P        = diag(P);
    
    if mod(i,updateRate/EKFUpdateRate) == 0
        
        x                       = runMPC(modelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
        [y0,tReal,yReal, uReal] = integrateStates(x,y0,t0,tf,t_sort,N,intdt,m,n,modelNumber,refTraj);
        yrefstore               = interp1(refTraj(:,1),refTraj(:,2:4),tReal)';
        
        stored(count).y     = yReal';
        stored(count).t     = tReal';
        stored(count).u     = uReal';
        stored(count).yref  = yrefstore;
        
        count = count + 1;
        
        tf = tf + updateRate;
        
    end
    
    t0 = t0 + EKFUpdateRate;
    
    u            = interp1(tReal(:,1),uReal(:,1:2),t0)';
    measurement  = interp1(tReal(:,1),yReal(:,1:3),t0)';
    measurement  = measurement + 0.01*rand(3,1);
    
end

plant_tout    = [plantData.t];
plant_yout    = [plantData.y];
plant_uout    = [plantData.u];
plant_yrefout = [plantData.yref];

save(plantFileName, 'plant_tout','plant_yout','plant_uout','plant_yrefout');

EKF_tout      = [EKFData.t];
EKF_Xout      = [EKFData.X];
EKF_Pout      = [EKFData.P];

save(EKFFileName, 'EKF_tout','EKF_Xout','EKF_Pout');