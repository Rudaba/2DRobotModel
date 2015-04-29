function [DS_data, MS_data, Coll_data, Pseud_data] = calcFinalError(errorLimit, initTime, finalTime, n, m, intdt, refTraj, y0)

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
    
    statePointsMS   = [5;10;50;100];
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
            
            xRef                      = interp1(refTraj(:,1),refTraj(:,2:end),DS_data{i}{j}{k}.time);
            
            DS_data{i}{j}{k}.errorVec = xRef(:,1:3) - DS_data{i}{j}{k}.states; %repmat(yRef,lengthStateVec,1);
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,1)) < errorLimit))
                DS_data{i}{j}{k}.xErrorTime     = nan;
            else
                index                       = find(abs(DS_data{i}{j}{k}.errorVec(:,1)) < errorLimit);
                DS_data{i}{j}{k}.xErrorTime = DS_data{i}{j}{k}.time(index(1));
            end
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,2)) < errorLimit))
                DS_data{i}{j}{k}.yErrorTime   = nan;
            else
                index                       = find(abs(DS_data{i}{j}{k}.errorVec(:,2)) < errorLimit);
                DS_data{i}{j}{k}.yErrorTime = DS_data{i}{j}{k}.time(index(1));
            end
            
            if isempty(find(abs(DS_data{i}{j}{k}.errorVec(:,3)) < errorLimit))
                DS_data{i}{j}{k}.psiErrorTime   = nan;
            else
                index   = find(abs(DS_data{i}{j}{k}.errorVec(:,3)) < errorLimit);
                DS_data{i}{j}{k}.psiErrorTime = DS_data{i}{j}{k}.time(index(1));
            end
            
            clear xRef
        end
    end
end

%For MS:

for i = 1:length(finalTime)
    
    for j = 1:length(sections)
        for k = 1:length(statePointsMS)
            
            for l = 1:sections(j)
                
                xRef                         = interp1(refTraj(:,1),refTraj(:,2:end),MS_data{i}{j}{k}.time{l});
                
                MS_data{i}{j}{k}.errorVec{l} = MS_data{i}{j}{k}.states{l} - xRef(:,1:3);
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,1)) < errorLimit))
                    MS_data{i}{j}{k}.xErrorTime{l}   = nan;
                else
                    index                       = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,1)) < errorLimit);
                    MS_data{i}{j}{k}.xErrorTime{l} = MS_data{i}{j}{k}.time{l}(index(1));
                end
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,2)) < errorLimit))
                    MS_data{i}{j}{k}.yErrorTime{l}   = nan;
                else
                    index   = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,2)) < errorLimit);
                    MS_data{i}{j}{k}.yErrorTime{l} = MS_data{i}{j}{k}.time{l}(index(1));
                end
                
                if isempty(find(abs(MS_data{i}{j}{k}.errorVec{l}(:,3)) < errorLimit))
                    MS_data{i}{j}{k}.psiErrorTime{l}   = nan;
                else
                    index  = find(abs(MS_data{i}{j}{k}.errorVec{l}(:,3)) < errorLimit);
                    MS_data{i}{j}{k}.psiErrorTime{l} = MS_data{i}{j}{k}.time{l}(index(1));
                end
                
                clear xRef
            end
        end
    end
end

%For Collocation:

for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        
        xRef                     = interp1(refTraj(:,1),refTraj(:,2:end),Coll_data{i}{j}.time);
        
        Coll_data{i}{j}.errorVec = Coll_data{i}{j}.states - xRef(:,1:3);
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,1)) < errorLimit))
            Coll_data{i}{j}.xErrorTime   = nan;
        else
            index   = find(abs(Coll_data{i}{j}.errorVec(:,1)) < errorLimit);
            Coll_data{i}{j}.xErrorTime = Coll_data{i}{j}.time(index(1));
        end
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,2)) < errorLimit))
            Coll_data{i}{j}.yErrorTime   = nan;
        else
            index   = find(abs(Coll_data{i}{j}.errorVec(:,2)) < errorLimit);
            Coll_data{i}{j}.yErrorTime = Coll_data{i}{j}.time(index(1));
        end
        
        if isempty(find(abs(Coll_data{i}{j}.errorVec(:,3)) < errorLimit))
            Coll_data{i}{j}.psiErrorTime   = nan;
        else
            index   = find(abs(Coll_data{i}{j}.errorVec(:,3)) < errorLimit);
            Coll_data{i}{j}.psiErrorTime = Coll_data{i}{j}.time(index(1));
        end
        
        clear xRef
    end
end

%For Pseudospectral:

for i = 1:length(finalTime)
    
    for j = 1:length(nodePoints)
        
        xRef                      = interp1(refTraj(:,1),refTraj(:,2:end),Pseud_data{i}{j}.time);
        
        Pseud_data{i}{j}.errorVec = Pseud_data{i}{j}.states - xRef(:,1:3);
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,1)) < errorLimit))
            Pseud_data{i}{j}.xErrorTime   = nan;
        else
            index   = find(abs(Pseud_data{i}{j}.errorVec(:,1)) < errorLimit);
            Pseud_data{i}{j}.xErrorTime = Pseud_data{i}{j}.time(index(1));
        end
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,2)) < errorLimit))
            Pseud_data{i}{j}.yErrorTime   = nan;
        else
            index   = find(abs(Pseud_data{i}{j}.errorVec(:,2)) < errorLimit);
            Pseud_data{i}{j}.yErrorTime = Pseud_data{i}{j}.time(index(1));
        end
        
        if isempty(find(abs(Pseud_data{i}{j}.errorVec(:,3)) < errorLimit))
            Pseud_data{i}{j}.psiErrorTime   = nan;
        else
            index   = find(abs(Pseud_data{i}{j}.errorVec(:,3)) < errorLimit);
            Pseud_data{i}{j}.psiErrorTime = Pseud_data{i}{j}.time(index(1));
        end
        
        clear xRef
    end
end
