function [X_Filter, P, z, S, innovation] = IMM_EKFPredictAndUpdate(X_Filter,P,mode,u,dt,measurement,Q,R_Noise)

global b

x   = X_Filter(1,1);
y   = X_Filter(2,1);
psi = X_Filter(3,1);

RR  = X_Filter(4,1);
RL  = X_Filter(5,1);

omegaR = u(1,1);
omegaL = u(2,1);

%Prediction
psiDot = RR/(2*b)*omegaR - RL/(2*b)*omegaL;
psi    = psi + psiDot*dt;

V      = RR/2*omegaR + RL/2*omegaL;

xDot   =  V*cos(psi);
x      = x + xDot*dt;


yDot   = V*sin(psi);
y      = y + yDot*dt;

F   = [1, 0, (-RR/2*omegaR*sin(psi) - RL/2*omegaL*sin(psi))*dt, (omegaR/2*cos(psi))*dt, (omegaL/2*cos(psi)) * dt;...
       0, 1, (RR/2*omegaR*cos(psi) + RL/2*omegaL*cos(psi))*dt, (omegaR/2*sin(psi))*dt, (omegaL/2*sin(psi)) * dt;...
       0, 0, 1, (omegaR/(2*b))*dt, (-omegaL/(2*b))*dt;...
       0, 0, 0, 1, 0;...
       0, 0, 0, 0, 1];

P          = Q + F * P *F';

% if mode == 1
%     RR = 2;
%     RL = 2;
% elseif mode == 2
%     RR = 0.5*2;
%     RL = 2;
% elseif mode == 3
%     RR = 2;
%     RL = 0.5*2;
% end

X_Filter   = [x;y;psi;RR;RL];

%Update
H           = eye(3,5);
z           = [x;y;psi];
innovation  =  measurement - z;
S           = H*P*H' + R_Noise;
K           = P * H' * inv(S);
X_Filter    = X_Filter + K * innovation;
P           = P - K*S*K';
