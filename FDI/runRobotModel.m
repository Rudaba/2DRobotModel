global refTraj N n m y0 t0 Hp x intdt t_sort MPCmodelNumber filterModelNumber
global b X_Filter MPCUpdateRate
%*****Define Simulation Parameters***
MPCmodelNumber      = 1; % LMPC = 1, NMPC = 2, NMPC with rates = 3
filterModelNumber   = 1; % EKF = 1, UKF = 2, IMM EKF = 3, IMM UKF = 4
plantFileName       = 'plantData_LMPC';
EKFFileName         = 'EKFData_LMPC';
count               = 1;

%*****Define Model Parameters*****
N   = 100;  % Number of collocation points for Direct Collocation and Pseudospectral

%*****Define Variable Parameters*****
MPCUpdateRate       = 0.1;
t0                  = 0;
Hp                  = 5;
intdt               = 0.01;
simTime             = 50;
FilterUpdateRate    = 0.01;
tf                  = MPCUpdateRate;

%****Define Robot Parameters*****
%Robot Constants
RR      = 2; %Radius of right tyre
RL      = 2; %Radius of left tyre
b       = 1; %Distance between centre of tyres

%*****Define Reference Trajectory*****
[refTraj] = calcRefTraj_circ;

%*****Define Initial Conditions*****
%This is initial nav robot state [x;y;psi]
y0          = refTraj(1,2:8)';    %This starts on the path
% y0          = [0;60;0;0;0]; %This is an offset from the path
u           = refTraj(1,7:8)';
measurement = y0(1:3);
% measurement = measurement + 0.01*rand(3,1);

%*****Define Constraints*****
constraints         = 0;
constraintValues    = [-inf,inf];
% constraintValues    = [-5,5];

%*****Intialisation*****
if MPCmodelNumber == 1
    
    % MPC
    [x,n,m,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseLinearMPC(N,constraintValues);
    
elseif MPCmodelNumber == 2
    
    % NMPC
    [x,n,m,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC(N,y0,u,constraintValues);
    
elseif MPCmodelNumber == 3
    
    % NMPC with rates
    [x,n,m,xlow,xupp,Flow,Fupp,iGfun,jGvar] = initialiseNonLinearMPC_withRates(N,y0,constraintValues);
    
end

if filterModelNumber == 1
    
    [X_Filter, P, Q, R_Noise] = initialiseEKF(y0(1:3),RR, RL);
    
elseif filterModelNumber == 2
    
    [X_Filter, P, Q, R_Noise] = initialiseUKF(y0,RR, RL);
    
end
%*****Set up optimisation*****
%SNOPT parameters
snsummary off;
snscreen on;

snseti('Verify level', -1);
% snseti('Derivative option', 0); % let SNOPT figure out jacobian
snseti('Derivative option', 2); % provide Jacobians
snseti('Major iterations',1000);

%*******************Start Simulation***************************************
%At time t0 = 0

[FilterData(1), X_Filter, P]            = runFilter(filterModelNumber,t0,X_Filter,P,u,FilterUpdateRate,measurement,Q,R_Noise);

x                                       = runMPC(MPCmodelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);

[plantData(1), y0, tReal, yReal, uReal] = integrateStates(x,y0,t0,t_sort,N,intdt,m,n,MPCmodelNumber,refTraj);

u                = interp1(tReal(:,1),uReal(:,1:2),t0,'pchip')';
u                = u + 0.01*rand(2,1);

measurement      = interp1(tReal(:,1),yReal(:,1:3),t0,'pchip')';
measurement      = measurement + 0.01*rand(3,1);

count                        = count + 1;
t0                           = t0 + FilterUpdateRate;

%Remaining simulation time
for i = 1:simTime/FilterUpdateRate
    
    [FilterData(i+1), X_Filter, P]  = runFilter(filterModelNumber,t0,X_Filter,P,u,FilterUpdateRate,measurement,Q,R_Noise);
    
    if mod(i,MPCUpdateRate/FilterUpdateRate) == 0
        
        x                                             = runMPC(MPCmodelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar);
        [plantData(count), y0, tReal, yReal, uReal]   = integrateStates(x,y0,t0,t_sort,N,intdt,m,n,MPCmodelNumber,refTraj);
        
        count = count + 1;
        
    end
    
    u                = interp1(tReal(:,1),uReal(:,1:2),t0,'pchip')';
    u                = u + 0.01*rand(2,1);
    
    measurement      = interp1(tReal(:,1),yReal(:,1:3),t0,'pchip')';
    measurement      = measurement + 0.01*rand(3,1);
    
    
    t0               = t0 + FilterUpdateRate;
    
end

plant_tout    = [plantData.t];
plant_yout    = [plantData.y];
plant_uout    = [plantData.u];
plant_yrefout = [plantData.yref];

save(plantFileName, 'plant_tout','plant_yout','plant_uout','plant_yrefout');

Filter_tout        = [FilterData.t];
Filter_Xout        = [FilterData.X];
Filter_Pout        = [FilterData.P];
Filter_Innovations = [FilterData.innovations];
Filter_S           = [FilterData.S];

save(EKFFileName, 'Filter_tout','Filter_Xout','Filter_Pout','Filter_Innovations','Filter_S');