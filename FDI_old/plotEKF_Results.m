data = load('EKFData_NMPC');

RR          = 2; 
RL          = 2; 
time        = [data.Filter_tout];
X           = [data.Filter_Xout];
P           = [data.Filter_Pout];
innovations = [data.Filter_Innovations];
S           = [data.Filter_S];
% time        = time(1:end-1)

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
plot(time,2*sqrt(S(1,:)),'--r')
hold on
plot(time,-2*sqrt(S(1,:)),'--r')
title('X innovations')

figure;
plot(time,innovations(2,:),'b')
hold on
plot(time,2*sqrt(S(2,:)),'--r')
hold on
plot(time,-2*sqrt(S(2,:)),'--r')
title('Y innovations')

figure;
plot(time,innovations(3,:),'b')
hold on
plot(time,2*sqrt(S(3,:)),'--r')
hold on
plot(time,-2*sqrt(S(3,:)),'--r')
title('Psi innovations')