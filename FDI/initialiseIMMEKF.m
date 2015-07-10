function [X_Filter, P_Filter, X_IMM, P_IMM, Q, R_Noise, transMatrix, modeProbs] = initialiseIMMEKF(y0,RR,RL)

%X_Filter and P_Filter represent the overall filter output,
%X_IMM and P_IMM represent the individual filter output

numberOfModels  = 3; 

X_Filter        = [y0;RR;RL];
P_Filter        = eye(5);
Q               = zeros(5,5,numberOfModels);
R_Noise         = eye(3); %There'll only ever be one measurement

X_IMM           = zeros(5,numberOfModels); 

X_IMM(:,1)      = [y0;RR;RL];
X_IMM(:,2)      = [y0;0.5*RR;RL];
X_IMM(:,3)      = [y0;RR;0.5*RL];

P_Filter(1,1)   = P_Filter(1,1)*(1e-2)^2; 
P_Filter(2,2)   = P_Filter(2,2)*(1e-2)^2;
P_Filter(3,3)   = P_Filter(3,3)*(1e-2)^2;
P_Filter(4,4)   = P_Filter(4,4)*(1e-3)^2;
P_Filter(5,5)   = P_Filter(5,5)*(1e-3)^2;

P_IMM           = zeros(5,5,numberOfModels); 
P_IMM(:,:,1)    = P_Filter;
P_IMM(:,:,2)    = P_Filter;
P_IMM(:,:,3)    = P_Filter;


Q               = zeros(5,5,numberOfModels); 
Q(1,1,1)        = (1e-2)^2; 
Q(2,2,1)        = (1e-2)^2;
Q(3,3,1)        = (1e-2)^2;
Q(4,4,1)        = (1e-3)^2;
Q(5,5,1)        = (1e-3)^2;

Q(1,1,2)        = (1e-2)^2; 
Q(2,2,2)        = (1e-2)^2;
Q(3,3,2)        = (1e-2)^2;
Q(4,4,2)        = (1e-3)^2;
Q(5,5,2)        = (1e-3)^2;

Q(1,1,3)        = (1e-2)^2; 
Q(2,2,3)        = (1e-2)^2;
Q(3,3,3)        = (1e-2)^2;
Q(4,4,3)        = (1e-3)^2;
Q(5,5,3)        = (1e-3)^2;

R_Noise(1,1)    = R_Noise(1,1)*(1e-2)^2; 
R_Noise(2,2)    = R_Noise(2,2)*(1e-2)^2;
R_Noise(3,3)    = R_Noise(3,3)*(1e-2)^2;

transMatrix     = [0.98, 0.01, 0.01; 0.01, 0.98, 0.01; 0.01, 0.01, 0.98];
modeProbs       = [1/3, 1/3, 1/3];

% transMatrix     = [1];
% modeProbs       = [1];
