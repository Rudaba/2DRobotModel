function [X_Filter,P, ymeas, S, innovation] = UKFUpdate(X_Filter,P,u,dt,measurement,Q,R_Noise)

global b

%Calculate sigma points
kapa            = 0.1;
na              = length(X_Filter); %length of states
N               = length(measurement); %length of measurements
sigma           = zeros(na,2*na+1);
sigma_meas      = zeros(N,2*na+1);
sigma(:,1)      = X_Filter;
sigma_meas(:,1) = X_Filter(1:3);
W               = zeros(1+2*na,1);
W(1,1)          = kapa/(na+kapa);

for i = 1:2*length(X_Filter)
    eqn = chol((na+kapa)*P);
    if i>=1 && i<= na
        sigma(:,i+1) = X_Filter + eqn(i,:)';
    else
        sigma(:,i+1) = X_Filter - eqn(i-na,:)';
    end
    W(i+1) = 1/(2*(na+kapa));
end

omegaR = u(1,1);
omegaL = u(2,1);

%Time update for each sigma point
for i = 1:2*na+1
    
    x   = sigma(1,i);
    y   = sigma(2,i);
    psi = sigma(3,i);
    RR  = sigma(4,i);
    RL  = sigma(5,i);
    
    psiDot          = RR/(2*b)*omegaR - RL/(2*b)*omegaL;
    sigma(3,i)      = psi + psiDot*dt;
    
    V               = RR/2*omegaR + RL/2*omegaL;
    
    xDot            =  V*cos(psi);
    sigma(1,i)      = x + xDot*dt;
    
    
    yDot            = V*sin(psi);
    sigma(2,i)      = y + yDot*dt;
    
    %Measurement predictions
    sigma_meas(1,i) = sigma(1,i);
    sigma_meas(2,i) = sigma(2,i);
    sigma_meas(3,i) = sigma(3,i);
    
end


%Predict states and Convariance forward
X_Filter    = zeros(na,1);
P           = zeros(na,na);
ymeas       = zeros(N,1);

for i = 1:2*na+1
    X_Filter = X_Filter + W(i)*sigma(:,i);
end

for i = 1:2*na+1
    P = P + W(i)*[sigma(:,i)-X_Filter]*[sigma(:,i)-X_Filter]';
end

P = P + Q;

for i = 1:2*na+1
    ymeas = ymeas + W(i)*sigma_meas(:,i);
end

%Update
Pxz = zeros(na,N);
Pzz = zeros(N,N);

for i = 1:2*na+1
    Pxz = Pxz + W(i)*[sigma(:,i)-X_Filter]*[sigma_meas(:,i)-ymeas]';
    Pzz = Pzz + W(i)*[sigma_meas(:,i)-ymeas]*[sigma_meas(:,i)-ymeas]';
end

S           = R_Noise + Pzz;
k           = Pxz*inv(S);
innovation  = measurement - ymeas;
correction  = k*innovation;
X_Filter    = X_Filter + correction;
P           = P - k*S*k';