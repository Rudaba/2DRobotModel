function [uMPC,tMPC]     = runMPC(modelNumber,xlow,xupp,Flow,Fupp,iGfun,jGvar)

if modelNumber == 1
    [uMPC,tMPC]       = MPC_directShooting(xlow,xupp,Flow,Fupp,iGfun,jGvar);
elseif modelNumber == 2
    [uMPC,tMPC]       = MPC_multipleShooting(xlow,xupp,Flow,Fupp,iGfun,jGvar);
elseif modelNumber == 3
    [uMPC,tMPC]       = MPC_collocation(xlow,xupp,Flow,Fupp,iGfun,jGvar);
elseif modelNumber == 4
    [uMPC,tMPC]       = MPC_pseudospectral(xlow,xupp,Flow,Fupp,iGfun,jGvar);
elseif modelNumber == 5
    [uMPC,tMPC]       = MPC_linear(xlow,xupp,Flow,Fupp,iGfun,jGvar);
end