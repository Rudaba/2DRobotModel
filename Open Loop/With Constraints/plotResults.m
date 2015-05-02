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


[DS_data, MS_data, Coll_data, Pseud_data] = calcFinalError(errorLimit, initTime, finalTime, n, m, intdt, refTraj, y0);

%Plot final errors
%***Direct Shooting***
controlPoints   = [5;25;50;100];
statePointsDS   = [5;25;50;100;200];
colorMarker     = {'*-b','*-r','*-g','*-m',};

%x
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData(k,:) = [statePointsDS(k),abs(DS_data{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,2),colorMarker{j},'MarkerSize',10)
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final x Error Direct Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final x Error Direct Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final x Error Direct Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nx')
    ylabel('Final x Error')
    legend('Nu = 5', 'Nu = 25', 'Nu = 50', 'Nu = 100')
end

%y
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData(k,:) = [statePointsDS(k),abs(DS_data{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,3),colorMarker{j})
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final y Error Direct Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final y Error Direct Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final y Error Direct Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nx')
    ylabel('Final y Error')
    legend('Nu = 5', 'Nu = 25', 'Nu = 50', 'Nu = 100')
end

%psi
for i = 1:length(finalTime)
    figure
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            plotData(k,:) = [statePointsDS(k),abs(DS_data{i}{j}{k}.errorVec(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,4),colorMarker{j})
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final psi Error Direct Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final psi Error Direct Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final psi Error Direct Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nx')
    ylabel('Final psi Error')
    legend('Nu = 5', 'Nu = 25', 'Nu = 50', 'Nu = 100')
end


%***Multiple Shooting***
statePointsMS   = [5;10;50;100];
sections        = [2,5,10];

%x
for i = 1:length(finalTime)
    figure
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData(k,:) = [statePointsDS(k),abs(MS_data{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,2),colorMarker{j})
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final x Error Multiple Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final x Error Multiple Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final x Error Multiple Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nu')
    ylabel('Final x Error')
    legend('M = 2', 'M = 5', 'M = 10')
end

%y
for i = 1:length(finalTime)
    figure
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData(k,:) = [statePointsDS(k),abs(MS_data{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,3),colorMarker{j})
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final y Error Multiple Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final y Error Multiple Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final y Error Multiple Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nu')
    ylabel('Final y Error')
    legend('M = 2', 'M = 5', 'M = 10')
end

%psi
for i = 1:length(finalTime)
    figure
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            plotData(k,:) = [statePointsDS(k),abs(MS_data{i}{j}{k}.errorVec{sections(j)}(end,:))];
        end
        
        plot(plotData(:,1),plotData(:,4),colorMarker{j})
        hold on
        
        clear plotData
    end
    
    if i == 1
        title('Final psi Error Multiple Shooting, Hp = 1sec, No Constraints')
    elseif i == 2
        title('Final psi Error Multiple Shooting, Hp = 5sec, No Constraints')
    elseif i == 3
        title('Final psi Error Multiple Shooting, Hp = 10sec, No Constraints')
    end
    
    xlabel('Nu')
    ylabel('Final psi Error')
    legend('M = 2', 'M = 5', 'M = 10')
end

%***Collocation***
nodePoints      = [5;10;50;100;200];

%x
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Coll_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,2),colorMarker{i})
    hold on
    
    clear plotData

end

title('Collocation x Final Error, No Constraints')
xlabel('N')
ylabel('Final x Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')

%y
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Coll_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,3),colorMarker{i})
    hold on
    
    clear plotData

end

title('Collocation y Final Error, No Constraints')
xlabel('N')
ylabel('Final y Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')

%psi
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Coll_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,4),colorMarker{i})
    hold on
    
    clear plotData

end

title('Collocation psi Final Error, No Constraints')
xlabel('N')
ylabel('Final y Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')


%***Pseudospectral***
nodePoints      = [5;10;50;100;200];

%x
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Pseud_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,2),colorMarker{i})
    hold on
    
    clear plotData

end

title('Pseudospectral x Final Error, No Constraints')
xlabel('N')
ylabel('Final x Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')

%y
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Pseud_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,3),colorMarker{i})
    hold on
    
    clear plotData

end

title('Pseudospectral y Final Error, No Constraints')
xlabel('N')
ylabel('Final y Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')

%psi
figure
for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        plotData(j,:) = [nodePoints(j),abs(Pseud_data{i}{j}.errorVec(end,:))];
    end
    
    plot(plotData(:,1),plotData(:,4),colorMarker{i})
    hold on
    
    clear plotData

end

title('Pseudospectral psi Final Error, No Constraints')
xlabel('N')
ylabel('Final psi Error')
legend('Hp = 1', 'Hp = 5', 'Hp = 10')
