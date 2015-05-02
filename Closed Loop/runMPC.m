function [x] = runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar)

A       = [];
iAfun   = [];
jAvar   = [];

if modelNumber == 1
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunMPC', A, iAfun, jAvar, iGfun, jGvar);
   
    
elseif modelNumber == 2
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunNMPC', A, iAfun, jAvar, iGfun, jGvar);
    
end