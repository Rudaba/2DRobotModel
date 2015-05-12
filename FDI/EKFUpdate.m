function [X_EKF,P] = EKFUpdate(X_EKF,P,u,dt,measurement,Q,R_Noise)
global b

x   = X_EKF(1,1);
y   = X_EKF(2,1);
psi = X_EKF(3,1);
R   = X_EKF(4,1);

omegaR = u(1,1);
omegaL = u(2,1);

%Prediction
psiDot = R/(2*b)*(omegaR - omegaL);
psi    = psi + psiDot*dt;

V      = R/2*(omegaR + omegaL);

xDot   =  V*cos(psi);
x      = x + xDot*dt;


yDot   = V*sin(psi);
y      = y + yDot*dt;

F   = [1, 0, (-R/2*omegaR*sin(psi) - R/2*omegaL*sin(psi))*dt, (omegaR/2*cos(psi) + omegaL/2*cos(psi)) * dt;...
       0, 1, (R/2*omegaR*cos(psi) + R/2*omegaL*cos(psi))*dt, (omegaR/2*sin(psi) + omegaL/2*sin(psi)) * dt;...
       0, 0, 1, (omegaR/(2*b) - omegaL/(2*b))*dt;...
       0, 0, 0, 1];
   
P       = Q + F * P *F'; 
X_EKF   = [x;y;psi,R];

%Update
H           = eye(3,4);
innovation  = [x;y;psi] - measurement;
S           = H*P*H' + R_Noise;
K           = P * H' * inv(S);
X_EKF       = X_EKF + K * innovation;
P           = P - K*S*K';
