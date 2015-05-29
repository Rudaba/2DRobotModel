function processMSdata(x,y0,t0,tf,M,Nui,intdt,m,n)
global refTraj
%Extract time, Controls and States

dti = (tf - t0) / (M);
dtu = dti / (Nui);
t0j = [t0:dti:tf-dti];
tfj = [t0+dti:dti:tf];

for j = 1:M
    t{j} = [t0j(j):dtu:tfj(j)];
end

% Extract states and controls from x vector
for j = 1:M
    ystart{j}(:, 1) = x((j-1)*n+1:j*n);
    yend{j}(:, 1) = x(M*n+((j-1)*n+1:j*n));
end

count = 0;
for j = 1:M
    for k = 1:m
        u{j}(:,k) = x(2*M*n+(j-1)*(Nui+1)+count+((k-1)*(Nui+1)+1:k*(Nui+1)));
    end
    count = j*(Nui+1);
end

for j = 1:M
    [y0,tReal{j},yReal{j},uReal{j}] = simulateRobotRK(y0,t{j},u{j},intdt,t{j}(1),t{j}(end));
end

%Plot results
figure;
for j = 1:M
    subplot(2,1,1)
    plot(tReal{j},uReal{j}(:,1),'-r'); hold on;
end
title('Controls MS, omegaR (top), omegaL (bottom)')

for j = 1:M
    subplot(2,1,2)
    plot(tReal{j},uReal{j}(:,2),'-b'); hold on;
end

figure;
for j = 1:M
    subplot(2,1,1)
    plot(tReal{j}(:,1),yReal{j}(:,1)); hold on;
    plot(t{j},ystart{j}(1,1),'*r'); hold on;
    plot(t{j},yend{j}(1,1),'*g')
end
title('Integrated States, x (top), y (bottom)')

for j = 1:M
    subplot(2,1,2)
    plot(tReal{j}(:,1),yReal{j}(:,2)); hold on;
    plot(t{j},ystart{j}(2,1),'*r'); hold on;
    plot(t{j},yend{j}(2,1),'*g')
end

figure
for j = 1:M
    plot(yReal{j}(:,1),yReal{j}(:,2),'b'); hold on;
    
    ref  = interp1(refTraj(:,1),refTraj(:,2:end),tReal{j},'pchip');
    hold on
    plot(ref(:,1),ref(:,2),'r')
    
end
title('x vs y')
legend('integrated','reference')