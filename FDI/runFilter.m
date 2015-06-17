function [FilterData, X_Filter, P_Filter, X_IMM, P_IMM, modeProbs] = runFilter(filterModelNumber,t0,X_Filter,P_Filter,X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs)

% X_IMM          = [];
% P_IMM          = [];
% modeProbs      = [];
% 

if filterModelNumber == 1
    
    [X_Filter,P_Filter, innovation,S] = EKFUpdate(X_Filter,P_Filter,u,FilterUpdateRate,measurement,Q,R_Noise);
    
    
elseif filterModelNumber == 2
    
    [X_Filter,P_Filter, innovation,S]= UKFUpdate(X_Filter,P_Filter,u,FilterUpdateRate,measurement,Q,R_Noise);
    
    
elseif filterModelNumber == 3
    
    [X_Filter,P_Filter, innovation,S, X_IMM, P_IMM, modeProbs]= IMMEKFUpdate(X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs);
end

FilterData.t              = t0;
FilterData.X              = X_Filter;
FilterData.P              = diag(P_Filter);
FilterData.innovations    = innovation;
if filterModelNumber ~= 3
    FilterData.S              = diag(S);
else
    FilterData.S              = S;
end
