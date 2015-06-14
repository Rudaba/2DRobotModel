function processMSdata(x,y0,t0,Hp,M,Nui,intdt,m,n)
global refTraj
%Extract time, Controls and States

tf  = t0 + Hp;
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

timeVector  = t{1}(1:Nui)';
uVector     = u{1}(1:Nui,:);

for j = 2:M
    
        timeVector = [timeVector;t{j}(1:Nui)'];
    
        uVector    = [uVector;u{j}(1:Nui,:)];
    
%     [y0,tReal{j},yReal{j},uReal{j}] = simulateRobotRK(y0,t{j}',u{j},intdt,t{j}(1),t{j}(end));
    
    %     startTime   = endTime;
    
end

timeVector = [timeVector;5];

uVector    = [uVector;u{end}(Nui,:)];

[y0,tReal,yReal,uReal] = simulateRobotRK(y0,timeVector,uVector,intdt,timeVector(1),timeVector(end));

%Plot Results
figure;
subplot(2,1,1)
plot(tReal,uReal(:,1),'r')
hold on
subplot(2,1,2)
plot(tReal,uReal(:,2),'b')

figure
plot(tReal,yReal(:,1))
for j = 1:M
    hold on
    plot(t{j},ystart{j}(1,1),'*r'); 
    hold on;
    plot(t{j},yend{j}(1,1),'*g')
end
title('x')

figure
plot(tReal,yReal(:,2))
for j = 1:M
    hold on
    plot(t{j},ystart{j}(2,1),'*r'); 
    hold on;
    plot(t{j},yend{j}(2,1),'*g')
end
title('y')

ref  = interp1(refTraj(:,1),refTraj(:,2:end),tReal,'pchip');
figure
plot(ref(:,1),ref(:,2),'r')
hold on
plot(yReal(:,1),yReal(:,2),'b')

% %Plot results
% figure;
% for j = 1:M
%     subplot(2,1,1)
%     plot(tReal{j},uReal{j}(:,1),'-r'); hold on;
% end
% title('Controls MS, omegaR (top), omegaL (bottom)')
% 
% for j = 1:M
%     subplot(2,1,2)
%     plot(tReal{j},uReal{j}(:,2),'-b'); hold on;
% end
% 
% figure;
% for j = 1:M
%     subplot(2,1,1)
%     plot(tReal{j}(:,1),yReal{j}(:,1)); hold on;
%     plot(t{j},ystart{j}(1,1),'*r'); hold on;
%     plot(t{j},yend{j}(1,1),'*g')
% end
% title('Integrated States, x (top), y (bottom)')
% 
% for j = 1:M
%     subplot(2,1,2)
%     plot(tReal{j}(:,1),yReal{j}(:,2)); hold on;
%     plot(t{j},ystart{j}(2,1),'*r'); hold on;
%     plot(t{j},yend{j}(2,1),'*g')
% end
% 
% figure
% for j = 1:M
%     plot(yReal{j}(:,1),yReal{j}(:,2),'b'); hold on;
%     
%     ref  = interp1(refTraj(:,1),refTraj(:,2:end),tReal{j},'pchip');
%     hold on
%     plot(ref(:,1),ref(:,2),'r')
%     
% end
% title('x vs y')
% legend('integrated','reference')