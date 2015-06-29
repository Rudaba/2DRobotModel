%No Constraints
model               = [1;2];
constraints         = 0;
constraintValues    = [-inf,inf];
% initialConditions   = [5,0,0;-10,7,0];
initialConditions   = [0,60,0];

run = 0;

for i = 1:length(model)
    clear globals
    
    for j = 1:size(initialConditions,1)
        fileString = strcat('Model_',int2str(model(i)),'_NC_IC_3');
        runRobotModel_sims(constraints, constraintValues, model(i), initialConditions(j,:)', fileString)
    end
    
    run = run + 1
    
end

%With Constraint 
model               = [1;2];
constraints         = 1;
constraintValues    = [-3,3];
% initialConditions   = [5,0,0;-10,7,0];
initialConditions   = [0,60,0];

for i = 1:length(model)
    clear globals
    
    for j = 1:size(initialConditions,1)
        fileString = strcat('Model_',int2str(model(i)),'_WC_IC_3');
        runRobotModel_sims(constraints, constraintValues, model(i), initialConditions(j,:)', fileString)
    end
    
    run = run + 1
    
end

