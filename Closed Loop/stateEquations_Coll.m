function yDots = stateEquations(stateVec,u)

global n

yDots = zeros(n, length(stateVec));

%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%Extract Data
psi     = stateVec(3,:);
y       = stateVec(2,:);
x       = stateVec(1,:);

omegaR  = u(1,:);
omegaL  = u(2,:);

v       = R*(omegaR+omegaL)/2;

%Solve DE's
psiDot      = R*(omegaR-omegaL)/(2*b);
yDot        = v.*sin(psi);
xDot        = v.*cos(psi);

yDots(1,:)  = xDot;
yDots(2,:)  = yDot;
yDots(3,:)  = psiDot;