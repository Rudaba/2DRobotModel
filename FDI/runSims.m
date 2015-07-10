%*****Simulation parameters*****
MPCModel    = [1;2];
filterModel = [1;2;3;4];
noRuns      = 1;
Run         = 0;

%*****Run Simulations*****
%No Fault no feedback
feedback        = 0;
faultOccurrence = 0;
faultScenario   = 0;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_NFNFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_NFNFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%No Fault with feedback
feedback        = 1;
faultOccurrence = 0;
faultScenario   = 0;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_NFWFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_NFWFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%LW 50% @ t = 20secs no feedback
feedback        = 0;
faultOccurrence = 1;
faultScenario   = 1;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S1NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S1NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%LW 50% @ t = 20secs with feedback
feedback        = 1;
faultOccurrence = 1;
faultScenario   = 1;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S1WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S1WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%RW 50% @ t = 20secs no feedback
feedback        = 0;
faultOccurrence = 1;
faultScenario   = 2;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S2NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S2NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%RW 50% @ t = 20secs with feedback
feedback        = 1;
faultOccurrence = 1;
faultScenario   = 2;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S2WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S2WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%LW 50% @ t = 30secs and RW 50% @ t = 15secs no feedback
feedback        = 0;
faultOccurrence = 1;
faultScenario   = 3;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S3NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S3NFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

%LW 50% @ t = 30secs and RW 50% @ t = 15secs with feedback
feedback        = 1;
faultOccurrence = 1;
faultScenario   = 3;

for i = 1:length(MPCModel)
    for j = 1:length(filterModel)
        for k = 1:noRuns
            plantFileName = strcat('PlantFile_S3WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            filterFileName = strcat('FilterFile_S3WFB_Model_',int2str(i),'_Filter_',int2str(j),'_Run_',int2str(k));
            runRobotModel_simulations(MPCModel(i), filterModel(j), plantFileName, filterFileName, feedback, faultOccurrence, faultScenario)
            Run = Run + 1
        end
    end
end

