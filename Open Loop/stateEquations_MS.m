function stateVec = stateEquations(stateVec,u,dt)

%Robot Constants
R       = 2; %Radius of tyres
b       = 1; %Distance between centre of tyres

%Extract Data
psi     = stateVec(3);
y       = stateVec(2);
x       = stateVec(1);

omegaR  = u(1);
omegaL  = u(2);

v       = R*(omegaR+omegaL)/2;

%Solve DE's
psiDot      = R*(omegaR-omegaL)/(2*b);
psi         = psi + psiDot * dt; 
yDot        = v*sin(psi);
y           = y + yDot * dt;
xDot        = v*cos(psi);
x           = x + xDot * dt;

stateVec    = [x;y;psi];