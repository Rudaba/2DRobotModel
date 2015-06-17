function [X_Filter, P_Filter, X_IMM, P_IMM, Q, R_Noise, transMatrix, modeProbs] = initialiseIMMEKF(y0,RR,RL)

%X_Filter and P_Filter represent the overall filter output,
%X_IMM and P_IMM represent the individual filter output

X_Filter        = [y0;RR;RL];
P_Filter        = eye(5);
Q               = cell(1,2);
R_Noise         = eye(3); %There'll only ever be one measurement

X_IMM           = cell(1,2); 

X_IMM{1}        = [y0;RR;0];
X_IMM{2}        = [y0;0;RL];

P_Filter(1,1)   = P_Filter(1,1)*(1e-3)^2; 
P_Filter(2,2)   = P_Filter(2,2)*(1e-3)^2;
P_Filter(3,3)   = P_Filter(3,3)*(1e-3)^2;
P_Filter(4,4)   = P_Filter(4,4)*(1e-2)^2;
P_Filter(5,5)   = P_Filter(5,5)*(1e-2)^2;

P_IMM           = cell(1,2); 
P_IMM{1}        = P_Filter;
P_IMM{1}(5,5)   = 0;
P_IMM{2}        = P_Filter;
P_IMM{2}(4,4)   = 0;

Q{1}            = zeros(5,5); 
Q{1}(1,1)       = (1e-1)^2; 
Q{1}(2,2)       = (1e-1)^2;
Q{1}(3,3)       = (1e-1)^2;
Q{1}(4,4)       = (1e-1)^2;
Q{1}(5,5)       = 0;

Q{2}            = zeros(5,5); 
Q{2}(1,1)       = (1e-1)^2; 
Q{2}(2,2)       = (1e-1)^2;
Q{2}(3,3)       = (1e-1)^2;
Q{2}(4,4)       = 0;
Q{2}(5,5)       = (1e-1)^2;

R_Noise(1,1)    = R_Noise(1,1)*(1e-2)^2; 
R_Noise(2,2)    = R_Noise(2,2)*(1e-2)^2;
R_Noise(3,3)    = R_Noise(3,3)*(1e-2)^2;

transMatrix     = [0.98, 0.02; 0.02, 0.98];
modeProbs       = [0.5, 0.5];
