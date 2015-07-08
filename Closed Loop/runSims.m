%No Constraints
model               = [2];
constraints         = 0;
constraintValues    = [-inf,inf];
initialConditions   = [5,0,0;-10,7,0;0,60,0];

run                 = 0;

for i = 1:length(model)
    clear globlas

    for j = 1:size(initialConditions,1)
        fileString = strcat('Model_',int2str(model(i)),'_NC_IC_',int2str(j));
        runRobotModel_simulations(constraints, constraintValues, model(i), initialConditions(j,:), fileString)
    run = run + 1

    end

end

%With Constraints
model               = [1,2];
constraints         = 1;
constraintValues    = [-3,3];
initialConditions   = [5,0,0;-10,7,0;0,60,0];

run                 = 0;

for i = 1:length(model)
    clear globlas
    
    for j = 1:size(initialConditions,1)
        fileString = strcat('Model_',int2str(model(i)),'_WC_IC_',int2str(j));
        runRobotModel_simulations(constraints, constraintValues, model(i), initialConditions(j,:), fileString)
        
        run = run + 1
    end
    
    
end