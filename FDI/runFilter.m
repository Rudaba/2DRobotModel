function [FilterData, X_Filter, P_Filter, X_IMM, P_IMM, modeProbs] = runFilter(filterModelNumber,t0,X_Filter,P_Filter,X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs)

if filterModelNumber == 1
    
    [X_Filter,P_Filter, innovation,S] = EKFUpdate(X_Filter,P_Filter,u,FilterUpdateRate,measurement,Q,R_Noise);
    
elseif filterModelNumber == 2
    
    [X_Filter,P_Filter, measurement, S, innovation]= UKFUpdate(X_Filter,P_Filter,u,FilterUpdateRate,measurement,Q,R_Noise);
        
elseif filterModelNumber == 3
    
    [X_Filter,P_Filter, innovation,S, X_IMM, P_IMM, modeProbs]= IMMEKFUpdate(X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs);
    
elseif filterModelNumber == 4
    
    [X_Filter,P_Filter, innovation,S, X_IMM, P_IMM, modeProbs]= IMMUKFUpdate(X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs);
    
end

FilterData.t              = t0;
FilterData.X              = X_Filter;
FilterData.P              = diag(P_Filter);
FilterData.innovations    = innovation;

if filterModelNumber == 1 || filterModelNumber == 2
    FilterData.S              = diag(S);
else
    FilterData.S              = S;
end
