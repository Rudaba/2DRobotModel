data = load('EKFData');

RR          = 2; 
RL          = 2; 
time        = [data.EKF_tout];
X           = [data.EKF_Xout];
P           = [data.EKF_Pout];
innovations = [data.EKF_Innovations];

figure;
plot(time,RR-X(4,:),'b')
hold on
plot(time,2*sqrt(P(4,:)),'--r')
hold on
plot(time,-2*sqrt(P(4,:)),'--r')
title('RR')

figure;
plot(time,RL-X(5,:),'b')
hold on
plot(time,2*sqrt(P(5,:)),'--r')
hold on
plot(time,-2*sqrt(P(5,:)),'--r')
title('RL')

figure;
plot(time,innovations(1,:),'b')
hold on
plot(time,2*sqrt(P(1,:)),'--r')
hold on
plot(time,-2*sqrt(P(1,:)),'--r')
title('X innovations')

figure;
plot(time,innovations(2,:),'b')
hold on
plot(time,2*sqrt(P(2,:)),'--r')
hold on
plot(time,-2*sqrt(P(2,:)),'--r')
title('Y innovations')

figure;
plot(time,innovations(3,:),'b')
hold on
plot(time,2*sqrt(P(3,:)),'--r')
hold on
plot(time,-2*sqrt(P(3,:)),'--r')
title('Psi innovations')