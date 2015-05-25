function [FilterData, X_Filter, P] = runFilter(filterModelNumber,t0,X_Filter,P,u,FilterUpdateRate,measurement,Q,R_Noise)

if filterModelNumber == 1
    
    [X_Filter,P, innovation,S] = EKFUpdate(X_Filter,P,u,FilterUpdateRate,measurement,Q,R_Noise);
    
elseif filterModelNumber == 2 
    
   [X_Filter,P, innovation,S]= UKFUpdate(X_Filter,P,u,FilterUpdateRate,measurement,Q,R_Noise);
    
end

FilterData.t              = t0;
FilterData.X              = X_Filter;
FilterData.P              = diag(P);
FilterData.innovations    = innovation;
FilterData.S              = diag(S);

