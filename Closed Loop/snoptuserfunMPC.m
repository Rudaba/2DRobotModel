function [F,Jac,tout,yout,uout] = snoptuserfunMPC(x)

global N t0 Hp y0 n m refTraj D_sort w t_sort

numconstr   = 1+n*(N+1) + n;
F           = zeros(numconstr,1);
y           = zeros(N+1,n); 
u           = zeros(N+1,m);
Eqn         = zeros(N+1,n);
tf          = t0 + Hp; 
factor      = (tf-t0)/2;

% Extract states and controls from x vector
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end
for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t            = ((tf-t0)/2*t_sort+(tf+t0)/2);
xRef         = interp1(refTraj(:,1),refTraj(:,2:end),t)';
ydots        = stateEquationsMPC(y, u, xRef)';

for index=n:-1:1
    Eqn(:,index) = 1./factor.*D_sort*y(:,index) - ydots(:,index);
end

[Mayer,Integral]    = integralCost(y',u');

F(1) = Mayer + sum(w.*Integral')*factor;%0.5*x'*H*x;%
for k = 1:n
    F(1+(k-1).*(N+1)+1:1+k.*(N+1)) = Eqn(:,k);
end


% initial conditions
F(1+n.*(N+1)+1) = y(1,1) + xRef(1,1) - y0(1);
F(1+n.*(N+1)+2) = y(1,2) + xRef(2,1) - y0(2);
F(1+n.*(N+1)+3) = y(1,3) + xRef(3,1) - y0(3);

%Terminal constraints
F(1+n.*(N+1)+4) = 0;
F(1+n.*(N+1)+5) = 0;
F(1+n.*(N+1)+6) = 0;

Jac=[];
