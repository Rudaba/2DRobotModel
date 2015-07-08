function [x] = runMPC(modelNumber,x,constraints,constraintValues,xlow,xupp,Flow,Fupp,iGfun,jGvar)
global t0 Hp t_sort refTraj N m n

A       = [];
iAfun   = [];
jAvar   = [];

if modelNumber == 1
    
    if constraints == 1
        
        t                  = (((t0+Hp)-t0)/2*t_sort+((t0+Hp)+t0)/2);
        
        xRef               = interp1(refTraj(:,1),refTraj(:,2:end),t);
        
        for i = 1:length(t)
            [omegaR0(i),omegaL0(i)]   = calcFeedforward(xRef(i,4),xRef(i,5));
        end
        
        for k = 1:m
            if k == 1
                nominal_omega = omegaR0;
            elseif k == 2
                nominal_omega = omegaL0;
            end
            
            
            xlow(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(1) - nominal_omega;
            xupp(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)),1) = constraintValues(2) - nominal_omega;
            
        end
    end
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunLinearMPC', A, iAfun, jAvar, iGfun, jGvar);
    
    
elseif modelNumber == 2
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunNonLinearMPC', A, iAfun, jAvar, iGfun, jGvar);
    
end