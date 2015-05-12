data = load('EKFData');

R       = 2; 
time    = [data.EKF_tout];
X       = [data.EKF_Xout];
P       = [data.EKF_Pout];

plot(time,R-X(4,:),'b')
hold on
plot(time,2*sqrt(P(4,:)),'--r')
hold on
plot(time,-2*sqrt(P(4,:)),'--r')