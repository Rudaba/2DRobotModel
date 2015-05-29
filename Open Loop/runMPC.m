function runMPC(MPCmodelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar,fileName,plotResults)

global t0 Hp Nu intdt M Nui y0 t_sort N m n

A       = [];
iAfun   = [];
jAvar   = [];

if MPCmodelNumber == 1
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunDirectShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    if plotResults == 1
        processDSdata(x,y0,t0,Hp,Nu,intdt,m)
    end
    
elseif MPCmodelNumber == 2
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunMultipleShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    if plotResults == 1
        processMSdata(x,y0,t0,Hp,M,Nui,intdt,m,n)
    end
    
elseif MPCmodelNumber == 3
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunCollocation', A, iAfun, jAvar, iGfun, jGvar);
    
    if plotResults == 1
        processCollData(x,y0,t0,Hp,N,intdt,m,n)
    end
    
elseif MPCmodelNumber == 4
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunPseudospectral', A, iAfun, jAvar, iGfun, jGvar);
    
    if plotResults == 1
         processPseudData(x,y0,t0,Hp,t_sort,N,intdt,m,n)
    end
    
    elseif MPCmodelNumber == 5
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunLinearMPC', A, iAfun, jAvar, iGfun, jGvar);
    
    if plotResults == 1
         processLinearMPCData(x,y0,t0,Hp,t_sort,N,intdt,m,n)
    end
    
    
end

save(fileName,'x')
