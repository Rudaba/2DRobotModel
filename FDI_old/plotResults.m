updateRate = 11;

Data_Model_1_NC_IC_1    = load('Model_1_NC_IC_1');
y1N1                     = [Data_Model_1_NC_IC_1.yout];
time1N1                  = [Data_Model_1_NC_IC_1.tout];
yref1N1                  = [Data_Model_1_NC_IC_1.yrefout];
u1N1                     = [Data_Model_1_NC_IC_1.uout];

Data_Model_1_NC_IC_2     = load('Model_1_NC_IC_2');
y1N2                     = [Data_Model_1_NC_IC_2.yout];
time1N2                  = [Data_Model_1_NC_IC_2.tout];
yref1N2                  = [Data_Model_1_NC_IC_2.yrefout];
u1N2                     = [Data_Model_1_NC_IC_2.uout];

Data_Model_1_NC_IC_3     = load('Model_1_NC_IC_3');
y1N3                     = [Data_Model_1_NC_IC_3.yout];
time1N3                  = [Data_Model_1_NC_IC_3.tout];
yref1N3                  = [Data_Model_1_NC_IC_3.yrefout];
u1N3                     = [Data_Model_1_NC_IC_3.uout];

Data_Model_1_WC_IC_1     = load('Model_1_WC_IC_1');
y1W1                     = [Data_Model_1_WC_IC_1.yout];
time1W1                  = [Data_Model_1_WC_IC_1.tout];
yref1W1                  = [Data_Model_1_WC_IC_1.yrefout];
u1W1                     = [Data_Model_1_WC_IC_1.uout];

Data_Model_1_WC_IC_2     = load('Model_1_WC_IC_2');
y1W2                     = [Data_Model_1_WC_IC_2.yout];
time1W2                  = [Data_Model_1_WC_IC_2.tout];
yref1W2                  = [Data_Model_1_WC_IC_2.yrefout];
u1W2                     = [Data_Model_1_WC_IC_2.uout];

Data_Model_1_WC_IC_3     = load('Model_1_WC_IC_3');
y1W3                     = [Data_Model_1_WC_IC_3.yout];
time1W3                  = [Data_Model_1_WC_IC_3.tout];
yref1W3                  = [Data_Model_1_WC_IC_3.yrefout];
u1W3                     = [Data_Model_1_WC_IC_3.uout];

Data_Model_2_NC_IC_1    = load('Model_2_NC_IC_1');
y2N1                     = [Data_Model_2_NC_IC_1.yout];
time2N1                  = [Data_Model_2_NC_IC_1.tout];
yref2N1                  = [Data_Model_2_NC_IC_1.yrefout];
u2N1                     = [Data_Model_2_NC_IC_1.uout];

Data_Model_2_NC_IC_2     = load('Model_2_NC_IC_2');
y2N2                     = [Data_Model_2_NC_IC_2.yout];
time2N2                  = [Data_Model_2_NC_IC_2.tout];
yref2N2                  = [Data_Model_2_NC_IC_2.yrefout];
u2N2                     = [Data_Model_2_NC_IC_2.uout];

Data_Model_2_NC_IC_3     = load('Model_2_NC_IC_3');
y2N3                     = [Data_Model_2_NC_IC_3.yout];
time2N3                  = [Data_Model_2_NC_IC_3.tout];
yref2N3                  = [Data_Model_2_NC_IC_3.yrefout];
u2N3                     = [Data_Model_2_NC_IC_3.uout];

Data_Model_2_WC_IC_1     = load('Model_2_WC_IC_1');
y2W1                     = [Data_Model_2_WC_IC_1.yout];
time2W1                  = [Data_Model_2_WC_IC_1.tout];
yref2W1                  = [Data_Model_2_WC_IC_1.yrefout];
u2W1                     = [Data_Model_2_WC_IC_1.uout];

Data_Model_2_WC_IC_2     = load('Model_2_WC_IC_2');
y2W2                     = [Data_Model_2_WC_IC_2.yout];
time2W2                  = [Data_Model_2_WC_IC_2.tout];
yref2W2                  = [Data_Model_2_WC_IC_2.yrefout];
u2W2                     = [Data_Model_2_WC_IC_2.uout];

Data_Model_2_WC_IC_3     = load('Model_2_WC_IC_3');
y2W3                     = [Data_Model_2_WC_IC_3.yout];
time2W3                  = [Data_Model_2_WC_IC_3.tout];
yref2W3                  = [Data_Model_2_WC_IC_3.yrefout];
u2W3                     = [Data_Model_2_WC_IC_3.uout];

%Plot NC IC 1 - [5,0,0]
figure
plot(yref1N1(1,1:updateRate:end),yref1N1(2,1:updateRate:end),'r')
hold on
plot(y1N1(1,1:updateRate:end),y1N1(2,1:updateRate:end),'b')
hold on
plot(y2N1(1,:),y2N1(2,:),'g')
hold on
plot(5,0,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - No Constraints Initial Conditions y0 = [5,0,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1N1(1:updateRate:end),u1N1(1,1:updateRate:end),'b')
hold on
plot(time2N1(1:updateRate:end),u2N1(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - No Constraints Initial Conditions y0 = [5,0,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-10,10])

subplot(2,1,2)
plot(time1N1(1:updateRate:end),u1N1(2,1:updateRate:end),'b')
hold on
plot(time2N1(1:updateRate:end),u2N1(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
ylim([-10,10])
legend('Linear','Nonlinear')

%Plot WC IC 1 - [5,0,0]
figure
plot(yref1W1(1,1:updateRate:end),yref1W1(2,1:updateRate:end),'r')
hold on
plot(y1W1(1,1:updateRate:end),y1W1(2,1:updateRate:end),'b')
hold on
plot(y2W1(1,1:updateRate:end),y2W1(2,1:updateRate:end),'g')
hold on
plot(5,0,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - With Constraints Initial Conditions y0 = [5,0,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1W1(1:updateRate:end),u1W1(1,1:updateRate:end),'b')
hold on
plot(time2W1(1:updateRate:end),u2W1(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - With Constraints Initial Conditions y0 = [5,0,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-5,5])

subplot(2,1,2)
plot(time1W1(1:updateRate:end),u1W1(2,1:updateRate:end),'b')
hold on
plot(time2W1(1:updateRate:end),u2W1(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
ylim([-5,5])
legend('Linear','Nonlinear')

%Plot NC IC 2 - [-10,7,0]
figure
plot(yref1N2(1,1:updateRate:end),yref1N2(2,1:updateRate:end),'r')
hold on
plot(y1N2(1,1:updateRate:end),y1N2(2,1:updateRate:end),'b')
hold on
plot(y2N2(1,1:updateRate:end),y2N2(2,1:updateRate:end),'g')
hold on
plot(-10,7,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - No Constraints Initial Conditions y0 = [-10,7,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1N2(1:updateRate:end),u1N2(1,1:updateRate:end),'b')
hold on
plot(time2N2(1:updateRate:end),u2N2(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - No Constraints Initial Conditions y0 = [-10,7,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-20,20])

subplot(2,1,2)
plot(time1N2(1:updateRate:end),u1N2(2,1:updateRate:end),'b')
hold on
plot(time2N2(1:updateRate:end),u2N2(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
ylim([-20,20])
legend('Linear','Nonlinear')

%Plot WC IC 2 - [-10,7,0]
figure
plot(yref1W2(1,1:updateRate:end),yref1W2(2,1:updateRate:end),'r')
hold on
plot(y1W2(1,1:updateRate:end),y1W2(2,1:updateRate:end),'b')
hold on
plot(y2W2(1,1:updateRate:end),y2W2(2,1:updateRate:end),'g')
hold on
plot(-10,7,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - With Constraints Initial Conditions y0 = [-10,7,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1W2(1:updateRate:end),u1W2(1,1:updateRate:end),'b')
hold on
plot(time2W2(1:updateRate:end),u2W2(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - With Constraints Initial Conditions y0 = [-10,7,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-5,5])

subplot(2,1,2)
plot(time1W2(1:updateRate:end),u1W2(2,1:updateRate:end),'b')
hold on
plot(time2W2(1:updateRate:end),u2W2(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
legend('Linear','Nonlinear')
ylim([-5,5])

%Plot NC IC 3 - [0,60,0]
figure
plot(yref1N3(1,1:updateRate:end),yref1N3(2,1:updateRate:end),'r')
hold on
plot(y1N3(1,1:updateRate:end),y1N3(2,1:updateRate:end),'b')
hold on
plot(y2N3(1,1:updateRate:end),y2N3(2,1:updateRate:end),'g')
hold on
plot(0,60,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - No Constraints Initial Conditions y0 = [0,60,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1N3(1:updateRate:end),u1N3(1,1:updateRate:end),'b')
hold on
plot(time2N3(1:updateRate:end),u2N3(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - No Constraints Initial Conditions y0 = [0,60,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-50,50])

subplot(2,1,2)
plot(time1N3(1:updateRate:end),u1N3(2,1:updateRate:end),'b')
hold on
plot(time2N3(1:updateRate:end),u2N3(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
legend('Linear','Nonlinear')
ylim([-50,50])

%Plot WC IC 3 - [0,60,0]
figure
plot(yref1W3(1,1:updateRate:end),yref1W3(2,1:updateRate:end),'r')
hold on
plot(y1W3(1,1:updateRate:end),y1W3(2,1:updateRate:end),'b')
hold on
plot(y2W3(1,1:updateRate:end),y2W3(2,1:updateRate:end),'g')
hold on
plot(0,60,'ok','LineWidth',2,'MarkerFaceColor','k')
title('Trajectory - With Constraints Initial Conditions y0 = [0,60,0]')
ylabel('y [m]')
xlabel('x [m]')
legend('Reference', 'Linear', 'Nonlinear','y0')

figure
subplot(2,1,1)
plot(time1W3(1:updateRate:end),u1W3(1,1:updateRate:end),'b')
hold on
plot(time2W3(1:updateRate:end),u2W3(1,1:updateRate:end),'g')
title('Angular Rates (Control Inputs) - With Constraints Initial Conditions y0 = [0,60,0]')
ylabel('OmegaR - u1 [rad/sec]')
legend('Linear','Nonlinear')
ylim([-5,5])

subplot(2,1,2)
plot(time1W3(1:updateRate:end),u1W3(2,1:updateRate:end),'b')
hold on
plot(time2W3(1:updateRate:end),u2W3(2,1:updateRate:end),'g')
ylabel('OmegaL - u2 [rad/sec]')
xlabel('Time [secs]')
legend('Linear','Nonlinear')
ylim([-5,5])

