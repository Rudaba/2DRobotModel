data = load('EKFData_NMPC');

m           = 3; %Number of models
RR          = 2;
RL          = 2;
time        = [data.Filter_tout];
X           = [data.Filter_Xout];
P           = [data.Filter_Pout];
innovations = [data.Filter_Innovations];
S           = [data.Filter_S];
obsStates   = size(innovations,1)/m;
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

for i = 1:m
    figure;
    plot(time,innovations(1+(i-1)*obsStates,:),'b')
    hold on
    plot(time,2*sqrt(S(1+(i-1)*obsStates,:)),'--r')
    hold on
    plot(time,-2*sqrt(S(1+(i-1)*obsStates,:)),'--r')
    title(strcat('X innovations ',' Model  ',int2str(i)))
    
    figure;
    plot(time,innovations(2+(i-1)*obsStates,:),'b')
    hold on
    plot(time,2*sqrt(S(2+(i-1)*obsStates,:)),'--r')
    hold on
    plot(time,-2*sqrt(S(2+(i-1)*obsStates,:)),'--r')
    title(strcat('Y innovations ',' Model  ',int2str(i)))
    
    figure;
    plot(time,innovations(3+(i-1)*obsStates,:),'b')
    hold on
    plot(time,2*sqrt(S(3+(i-1)*obsStates,:)),'--r')
    hold on
    plot(time,-2*sqrt(S(3+(i-1)*obsStates,:)),'--r')
    title(strcat('Psi innovations ',' Model  ',int2str(i)))
end