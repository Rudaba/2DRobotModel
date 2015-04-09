initTime    = 0;
finalTime   = [1; 5; 10];

%Sims for direct shooting
for i = 1:length(finalTime)
    clear globals
    nodePoints      = 50;
    controlPoints   = [5];%;25;50;100];
    statePointsDS   = [5];%;25;50;100;200];
    statePointsMS   = 10;
    sections        = 5;
    model           = 1;
    
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_Nu_',int2str(controlPoints(j)),'_Nx_',int2str(statePointsDS(k)));
            runRobotModel_sims(model,initTime,finalTime(i),statePointsDS(k),controlPoints(j),sections,statePointsMS,nodePoints,fileName);
        end
    end
end

%Sims for multiple shooting
for i = 1:length(finalTime)
    clear globals
    nodePoints      = 50;
    controlPoints   = 50;
    statePointsDS   = 100;
    statePointsMS   = [5];%;10;50];
    sections        = [2];%,5,10];
    model           = 2;
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_M_',int2str(sections(j)),'_Nui_',int2str(statePointsMS(k)));
            runRobotModel_sims(model,initTime,finalTime(i),statePointsDS,controlPoints,sections(j),statePointsMS(k),nodePoints,fileName);
        end
    end
end

%Sims for collocation
for i = 1:length(finalTime)
    clear globals
    nodePoints      = [5];%;10;50;100;200];
    controlPoints   = 50;
    statePointsDS   = 100;
    statePointsMS   = 10;
    sections        = 5;
    model           = 3;
    
    for j = 1:length(nodePoints)
        fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_N_',int2str(nodePoints(j)));
        runRobotModel_sims(model,initTime,finalTime(i),statePointsDS,controlPoints,sections,statePointsMS,nodePoints(j),fileName);
    end
end

%Sims for pseudospectral
for i = 1:length(finalTime)
    clear globals
    nodePoints      = [5];%;10;50;100;200];
    controlPoints   = 50;
    statePointsDS   = 100;
    statePointsMS   = 10;
    sections        = 5;
    model           = 4;
    
    for j = 1:length(nodePoints)
        fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_N_',int2str(nodePoints(j)));
        runRobotModel_sims(model,initTime,finalTime(i),statePointsDS,controlPoints,sections,statePointsMS,nodePoints(j),fileName);
    end
end