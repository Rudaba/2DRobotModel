function [refTraj] =  calcRefTrajectory

time    = [0:50]';
x       = [0:50]';
y       = 5*ones(51,1);
psi     = 0*ones(51,1);
V       = ones(51,1);
psiDot  = zeros(51,1);

xRef    = [time,x,y,psi,V,psiDot];


for i = 1:length(time)
    
    [omegaR0,omegaL0] = calcFeedforward(V(i,1), psiDot(i,1));
    
    omegaR(i) = omegaR0;
    omegaL(i) = omegaL0;
    
end

refTraj = [xRef,omegaR',omegaL'];