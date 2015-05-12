function [X_EKF, P, Q, R_Noise] = initialiseEKF(y0,R)

X_EKF           = [y0;R];
P               = eye(4);
Q               = eye(4);
R_Noise         = eye(3);

P(1,1)          = P(1,1)*(1e-3)^2; 
P(2,2)          = P(2,2)*(1e-3)^2;
P(3,3)          = P(3,3)*(1e-3)^2;
P(4,4)          = P(4,4)*(1e-3)^2;

Q(1,1)          = Q(1,1)*(1e-1)^2; 
Q(2,2)          = Q(2,2)*(1e-1)^2;
Q(3,3)          = Q(3,3)*(1e-1)^2;
Q(4,4)          = Q(4,4)*(1e-1)^2;

R_Noise(1,1)    = R_Noise(1,1)*(1e-3)^2; 
R_Noise(2,2)    = R_Noise(2,2)*(1e-3)^2;
R_Noise(3,3)    = R_Noise(3,3)*(1e-3)^2;
