%Calculate Final Error
global R b

initTime    = 0;
finalTime   = [1; 5; 10];
n           = 3;             % Number of states
m           = 2;             % Number of controls states
intdt       = 0.01;          %Integration time step
y0          = [-0.5;5+1;0];  %This is initial nav robot state [x;y;psi]
R           = 2;
b           = 1;
yRef        = [0,5,0];


%********************Gather all data*********************
%Sims for direct shooting
for i = 1:length(finalTime)
    
    controlPoints   = [5;25;50;100];
    statePointsDS   = [5;25;50;100;200];
    model           = 1;
    
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            
            fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_Nu_',int2str(controlPoints(j)),'_Nx_',int2str(statePointsDS(k)));
            
            data                = load(fileName);
            [tReal, yReal, u]  = processDSdata(data.x,y0,initTime,finalTime(i),controlPoints(j),intdt,m);
            
            DS_data{i}{j}{k}.time   = tReal;
            DS_data{i}{j}{k}.states = yReal;
            DS_data{i}{j}{k}.u      = u;
            
            clear tReal yReal u data
        end
    end
end

%Sims for multiple shooting
for i = 1:length(finalTime)
    
    statePointsMS   = [5;10;50];
    sections        = [2,5,10];
    model           = 2;
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            
            fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_M_',int2str(sections(j)),'_Nui_',int2str(statePointsMS(k)));
            data                = load(fileName);
            [tReal, yReal, u]  = processMSdata(data.x,y0,initTime,finalTime(i),sections(j),statePointsMS(k),intdt,m,n);
            
            MS_data{i}{j}{k}.time        = tReal;
            MS_data{i}{j}{k}.states      = yReal;
            MS_data{i}{j}{k}.u           = u;
            
            clear tReal yReal u data
        end
    end
end

%Sims for collocation
for i = 1:length(finalTime)
    nodePoints      = [5;10;50;100;200];
    model           = 3;
    
    for j = 1:length(nodePoints)
        
        fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_N_',int2str(nodePoints(j)));
        data               = load(fileName);
        [tReal, yReal, u]  = processCollData(data.x,y0,initTime,finalTime(i),nodePoints(j),intdt,m,n);
        
        Coll_data{i}{j}.time        = tReal;
        Coll_data{i}{j}.states      = yReal;
        Coll_data{i}{j}.u           = u;
        
        clear tReal yReal u data
        
    end
end

%Sims for pseudospectral
for i = 1:length(finalTime)
    clear globals
    nodePoints      = [5;10;50;100;200];
    model           = 4;
    
    for j = 1:length(nodePoints)
        
        fileName = strcat('Model_',int2str(model),'_tf_',int2str(finalTime(i)),'_N_',int2str(nodePoints(j)));
        data               = load(fileName);
        [t_sort,w]         = LegendreNodesAndWeights(nodePoints(j));
        [tReal, yReal, u]  = processPseudData(data.x,y0,initTime,finalTime(i),t_sort,nodePoints(j),intdt,m,n);
        
        Pseud_data{i}{j}.time        = tReal;
        Pseud_data{i}{j}.states      = yReal;
        Pseud_data{i}{j}.u           = u;
        
        clear tReal yReal u data t_sort
        
    end
end
%********************************************************************

%******************Calculate Final Error*****************************
%For DS:

for i = 1:length(finalTime)
    
    for j = 1:length(controlPoints)
        for k = 1:length(statePointsDS)
            
            lengthStateVec = size(DS_data{i}{j}{k}.states,1);
            
            DS_data{i}{j}{k}.errorVec = DS_data{i}{j}{k}.states - repmat(yRef,lengthStateVec,1);
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,1)) < 0.001))
                DS_data{i}{j}{k}.xError   = nan;
            else
                DS_data{i}{j}{k}.xError   = find(abs(DS_data{i}{j}{k}.errorVec(:,1)) < 0.001);
            end
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,2)) < 0.001))
                DS_data{i}{j}{k}.yError   = nan;
            else
                DS_data{i}{j}{k}.yError   = find(abs(DS_data{i}{j}{k}.errorVec(:,2)) < 0.001);
            end
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,3)) < 0.001))
                DS_data{i}{j}{k}.psiError   = nan;
            else
                DS_data{i}{j}{k}.psiError   = find(abs(DS_data{i}{j}{k}.errorVec(:,3)) < 0.001);
            end
            
            clear lengthStateVec
        end
    end
end

%For MS:

for i = 1:length(finalTime)
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            
            for l = 1:sections(j)
                
                lengthStateVec = size(MS_data{i}{j}{k}.states{l},1);
                
                MS_data{i}{j}{k}.errorVec{l} = MS_data{i}{j}{k}.states{l} - repmat(yRef,lengthStateVec,1);
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,1)) < 0.001))
                    MS_data{i}{j}{k}.xError{l}   = nan;
                else
                    MS_data{i}{j}{k}.xError{l}   = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,1)) < 0.001);
                end
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,2)) < 0.001))
                    MS_data{i}{j}{k}.yError{l}   = nan;
                else
                    MS_data{i}{j}{k}.yError{l}   = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,2)) < 0.001);
                end
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,3)) < 0.001))
                    MS_data{i}{j}{k}.psiError{l}   = nan;
                else
                    MS_data{i}{j}{k}.psiError{l}   = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,3)) < 0.001);
                end
                
                clear lengthStateVec
            end
        end
    end
end

%For Collocation:

for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        
        lengthStateVec = size(Coll_data{i}{j}.states,1);
        
        Coll_data{i}{j}.errorVec = Coll_data{i}{j}.states - repmat(yRef,lengthStateVec,1);
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,1)) < 0.001))
            Coll_data{i}{j}.xError   = nan;
        else
            Coll_data{i}{j}.xError   = find(abs(Coll_data{i}{j}.errorVec(:,1)) < 0.001);
        end
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,2)) < 0.001))
            Coll_data{i}{j}.yError   = nan;
        else
            Coll_data{i}{j}.yError   = find(abs(Coll_data{i}{j}.errorVec(:,2)) < 0.001);
        end
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,3)) < 0.001))
            Coll_data{i}{j}.psiError   = nan;
        else
            Coll_data{i}{j}.psiError   = find(abs(Coll_data{i}{j}.errorVec(:,3)) < 0.001);
        end
        
        clear lengthStateVec
    end
end

%For Pseudospectral:

for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        
        lengthStateVec = size(Pseud_data{i}{j}.states,1);
        
        Pseud_data{i}{j}.errorVec = Pseud_data{i}{j}.states - repmat(yRef,lengthStateVec,1);
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,1)) < 0.001))
            Pseud_data{i}{j}.xError   = nan;
        else
            Pseud_data{i}{j}.xError   = find(abs(Pseud_data{i}{j}.errorVec(:,1)) < 0.001);
        end
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,2)) < 0.001))
            Pseud_data{i}{j}.yError   = nan;
        else
            Pseud_data{i}{j}.yError   = find(abs(Pseud_data{i}{j}.errorVec(:,2)) < 0.001);
        end
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,3)) < 0.001))
            Pseud_data{i}{j}.psiError   = nan;
        else
            Pseud_data{i}{j}.psiError   = find(abs(Pseud_data{i}{j}.errorVec(:,3)) < 0.001);
        end
        
        clear lengthStateVec
    end
end
