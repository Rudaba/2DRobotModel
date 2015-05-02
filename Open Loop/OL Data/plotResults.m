global R b

initTime    = 0;
finalTime   = [1; 5; 10];
n           = 3;             % Number of states
m           = 2;             % Number of controls states
intdt       = 0.01;          % Integration time step
y0          = [-0.5;5+1;0];  % This is initial nav robot state [x;y;psi]
R           = 2;
b           = 1;
refTraj     = [[0:50]',[0:50]',5*ones(51,1),0*ones(51,1),ones(51,1)];
errorLimit  = 0.1;


[DS_data_NC, MS_data_NC, Coll_data_NC, Pseud_data_NC] = calcFinalError(errorLimit, initTime, finalTime, n, m, intdt, refTraj, y0, 1);

[DS_data_WC, MS_data_WC, Coll_data_WC, Pseud_data_WC] = calcFinalError(errorLimit, initTime, finalTime, n, m, intdt, refTraj, y0, 0);

%Plot final errors
%***Direct Shooting***
controlPoints   = [5;25;50;100];
statePointsDS   = [5;25;50;100;200];
colorMarker_NC     = {'*-b','*-r','*-g','*-m',};
colorMarker_WC     = {'o--b','o--r','o--g','o--m',};

%x
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData_NC(k,:) = [statePointsDS(k),abs(DS_data_NC{i}{j}{k}.errorVec(end,:))];
            plotData_WC(k,:) = [statePointsDS(k),abs(DS_data_WC{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,2),colorMarker_NC{j},'MarkerSize',10)
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,2),colorMarker_WC{j},'MarkerSize',10)
        
        clear plotData_NC plotData_NC
    end
    
    if i == 1
        title('Final x Error Direct Single Shooting, Hp = 1sec')
    elseif i == 2
        title('Final x Error Direct Single Shooting, Hp = 5secs')
    elseif i == 3
        title('Final x Error Direct Single Shooting, Hp = 10secs')
    end
    
    xlabel('Nx')
    ylabel('Final x Error [m]')
    legend('Nu = 5 No Constraints', 'Nu = 5 With Constraints','Nu = 25 No Constraints', 'Nu = 25 With Constraints',...
        'Nu = 50 No Constraints', 'Nu = 50 With Constraints', 'Nu = 100 No Constraints', 'Nu = 100 With Constraints')
end

%y
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData_NC(k,:) = [statePointsDS(k),abs(DS_data_NC{i}{j}{k}.errorVec(end,:))];
            plotData_WC(k,:) = [statePointsDS(k),abs(DS_data_WC{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,3),colorMarker_NC{j})
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,3),colorMarker_WC{j})
        
        clear plotData_NC plotData_WC
    end
    
    if i == 1
        title('Final y Error Direct Single Shooting, Hp = 1sec')
    elseif i == 2
        title('Final y Error Direct Single Shooting, Hp = 5secs')
    elseif i == 3
        title('Final y Error Direct Single Shooting, Hp = 10secs')
    end
    
    xlabel('Nx')
    ylabel('Final y Error [m]')
    legend('Nu = 5 No Constraints', 'Nu = 5 With Constraints','Nu = 25 No Constraints', 'Nu = 25 With Constraints',...
        'Nu = 50 No Constraints', 'Nu = 50 With Constraints', 'Nu = 100 No Constraints', 'Nu = 100 With Constraints')
end

%psi
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData_NC(k,:) = [statePointsDS(k),abs(DS_data_NC{i}{j}{k}.errorVec(end,:))];
            plotData_WC(k,:) = [statePointsDS(k),abs(DS_data_WC{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,4),colorMarker_NC{j})
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,4),colorMarker_WC{j})
        
        clear plotData_NC plotData_WC
    end
    
    if i == 1
        title('Final psi Error Direct Single Shooting, Hp = 1sec')
    elseif i == 2
        title('Final psi Error Direct Single Shooting, Hp = 5secs')
    elseif i == 3
        title('Final psi Error Direct Single Shooting, Hp = 10secs')
    end
    
    xlabel('Nx')
    ylabel('Final psi Error [rad]')
    legend('Nu = 5 No Constraints', 'Nu = 5 With Constraints','Nu = 25 No Constraints', 'Nu = 25 With Constraints',...
        'Nu = 50 No Constraints', 'Nu = 50 With Constraints', 'Nu = 100 No Constraints', 'Nu = 100 With Constraints')
end


%***Multiple Shooting***
statePointsMS   = [5;10;50;100];
sections        = [2,5,10];

%x
for i = 1:length(finalTime)
    figure
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData_NC(k,:) = [statePointsMS(k),abs(MS_data_NC{i}{j}{k}.errorVec{sections(j)}(end,:))];
            plotData_WC(k,:) = [statePointsMS(k),abs(MS_data_WC{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,2),colorMarker_NC{j})
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,2),colorMarker_WC{j})
        
        clear plotData_NC plotData_WC
    end
    
    if i == 1
        title('Final x Error Multiple Shooting, Hp = 1sec')
    elseif i == 2
        title('Final x Error Multiple Shooting, Hp = 5secs')
    elseif i == 3
        title('Final x Error Multiple Shooting, Hp = 10secs')
    end
    
    xlabel('Nu')
    ylabel('Final x Error [m]')
    legend('M = 2 No Constraints', 'M = 2 With Constraints', 'M = 5 No Constraints','M = 5 With Constraints',...
        'M = 10 No Constraints', 'M = 10 With Constraints')
end

%y
for i = 1:length(finalTime)
    figure
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData_NC(k,:) = [statePointsMS(k),abs(MS_data_NC{i}{j}{k}.errorVec{sections(j)}(end,:))];
            plotData_WC(k,:) = [statePointsMS(k),abs(MS_data_WC{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,3),colorMarker_NC{j})
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,3),colorMarker_WC{j})
        
        clear plotData_NC plotData_WC
    end
    
    if i == 1
        title('Final y Error Multiple Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final y Error Multiple Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final y Error Multiple Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nu')
    ylabel('Final y Error [m]')
    legend('M = 2 No Constraints', 'M = 2 With Constraints', 'M = 5 No Constraints','M = 5 With Constraints',...
        'M = 10 No Constraints', 'M = 10 With Constraints')
end

%psi
for i = 1:length(finalTime)
    figure
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData_NC(k,:) = [statePointsMS(k),abs(MS_data_NC{i}{j}{k}.errorVec{sections(j)}(end,:))];
            plotData_WC(k,:) = [statePointsMS(k),abs(MS_data_WC{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData_NC(:,1),plotData_NC(:,4),colorMarker_NC{j})
        hold on
        plot(plotData_WC(:,1),plotData_WC(:,4),colorMarker_WC{j})
        
        clear plotData
    end
    
    if i == 1
        title('Final psi Error Multiple Shooting, Hp = 1sec')
    elseif i == 2
        title('Final psi Error Multiple Shooting, Hp = 5secs')
    elseif i == 3
        title('Final psi Error Multiple Shooting, Hp = 10secs')
    end
    
    xlabel('Nu')
    ylabel('Final psi Error [rad]')
    legend('M = 2 No Constraints', 'M = 2 With Constraints', 'M = 5 No Constraints','M = 5 With Constraints',...
        'M = 10 No Constraints', 'M = 10 With Constraints')
end

%***Collocation***
nodePoints      = [5;10;50;100;200];

%x
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Coll_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Coll_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,2),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,2),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
end
title('Final x Error Collocation')
xlabel('N')
ylabel('Final x Error [m]')
legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')


%y
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Coll_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Coll_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,3),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,3),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
end
title('Final y Error Collocation')
xlabel('N')
ylabel('Final y Error [m]')

legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')


%psi
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Coll_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Coll_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,4),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,4),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
end
title('Final psi Error Collocation')
xlabel('N')
ylabel('Final psi [rad] Error')

legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')


%***Pseudospectral***
nodePoints      = [5;10;50;100;200];

%x
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Pseud_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Pseud_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,2),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,2),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
    
end
title('Final x Error Pseudospectral')
xlabel('N')
ylabel('Final x Error [m]')

legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')


%y
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Pseud_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Pseud_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,3),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,3),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
end
title('Final y Error Pseudospectral')
xlabel('N')
ylabel('Final y Error [m]')

legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')


%psi
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData_NC(j,:) = [nodePoints(j),abs(Pseud_data_NC{i}{j}.errorVec(end,:))];
        plotData_WC(j,:) = [nodePoints(j),abs(Pseud_data_WC{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData_NC(:,1),plotData_NC(:,4),colorMarker_NC{i})
    hold on
    plot(plotData_WC(:,1),plotData_WC(:,4),colorMarker_WC{i})
    
    clear plotData_NC plotData_WC
    
end
title('Final psi Error Pseudospectral')
xlabel('N')
ylabel('Final psi Error [rad]')


legend('Hp = 1sec No Constraints', 'Hp = 1sec With Constraints',...
    'Hp = 5secs No Constraints', 'Hp = 5secs With Constraints',...
    'Hp = 10secs No Constraints', 'Hp = 10secs With Constraints')

