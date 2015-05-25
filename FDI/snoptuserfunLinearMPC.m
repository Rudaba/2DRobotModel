function [F,Jac,tout,yout,uout] = snoptuserfunLinearMPC(x)

global N t0 Hp y0 n m refTraj D_sort w t_sort

numconstr   = 1+n*(N+1) + 2*(n) + m;
F           = zeros(numconstr,1);
y           = zeros(N+1,n); 
u           = zeros(N+1,m);
Eqn         = zeros(N+1,n);
tf          = t0 + Hp; 
factor      = (tf-t0)/2;

RR = 2;
RL = 2;


% Extract states and controls from x vector
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end
for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

t            = ((tf-t0)/2*t_sort+(tf+t0)/2);
xRef         = interp1(refTraj(:,1),refTraj(:,2:end),t)';
tRef         = refTraj(:,1);
[yDots, dv, dpsiDot, df1_dx, df2_dx, df3_dx]        = stateEquationsLinearMPC(y, u, t, tRef,xRef);

ydots = yDots';

for index=n:-1:1
    Eqn(:,index) = 1./factor.*D_sort*y(:,index) - ydots(:,index);
end

[Mayer, Integral, dC_dx]   = integralCost(y',u',dv',dpsiDot',xRef,RR,RL);

F(1) = Mayer + sum(w.*Integral')*factor;%0.5*x'*H*x;%
for k = 1:n
    F(1+(k-1).*(N+1)+1:1+k.*(N+1)) = Eqn(:,k);
end


% initial conditions
F(1+n.*(N+1)+1) = y(1,1) + xRef(1,1) - y0(1);
F(1+n.*(N+1)+2) = y(1,2) + xRef(2,1) - y0(2);
F(1+n.*(N+1)+3) = y(1,3) + xRef(3,1) - y0(3);

F(1+n.*(N+1)+4) = u(1,1) + xRef(6,1) - y0(4);
F(1+n.*(N+1)+5) = u(1,2) + xRef(7,1) - y0(5);

%Terminal constraints
F(1+n.*(N+1)+6) = 0;
F(1+n.*(N+1)+7) = 0;
F(1+n.*(N+1)+8) = 0;

% F(1+n.*(N+1)+9) = 0;
% F(1+n.*(N+1)+10)= 0;

Jac = zeros(1+n*(N+1)+2*(n) + m,(n+m)*(N+1));

%Cost Jacobian
Jac(1, 1:(N+1))      = dC_dx(1,:)'.*w*factor;
Jac(1, (N+2):2*N+2)  = dC_dx(2,:)'.*w*factor;
Jac(1, 2*N+3: 3*N+3) = dC_dx(3,:)'.*w*factor;
Jac(1, 3*N+4:4*N+4)  = dC_dx(4,:)'.*w*factor;
Jac(1, 4*N+5:5*N+5)  = dC_dx(5,:)'.*w*factor;

%x Jacobian
Jac(2:N+2, 1:(N+1))      = 1./factor.*D_sort - diag(df1_dx(1,:));
Jac(2:N+2, (N+2):2*N+2)  = -diag(df1_dx(2,:));
Jac(2:N+2, 2*N+3: 3*N+3) = -diag(df1_dx(3,:));
Jac(2:N+2, 3*N+4:4*N+4)  = -diag(df1_dx(4,:));
Jac(2:N+2, 4*N+5:5*N+5)  = -diag(df1_dx(5,:));

%y Jacobian
Jac(N+3:2*N+3, 1:(N+1))      = - diag(df2_dx(1,:));
Jac(N+3:2*N+3, (N+2):2*N+2)  = 1./factor.*D_sort - diag(df2_dx(2,:));
Jac(N+3:2*N+3, 2*N+3: 3*N+3) = - diag(df2_dx(3,:));
Jac(N+3:2*N+3, 3*N+4:4*N+4)  = - diag(df2_dx(4,:));
Jac(N+3:2*N+3, 4*N+5:5*N+5)  = - diag(df2_dx(5,:));

%psi Jacobian
Jac(2*N+4:3*N+4, 1:(N+1))      = - diag(df3_dx(1,:));
Jac(2*N+4:3*N+4, (N+2):2*N+2)  = - diag(df3_dx(2,:));
Jac(2*N+4:3*N+4, 2*N+3: 3*N+3) = 1./factor.*D_sort - diag(df3_dx(3,:));
Jac(2*N+4:3*N+4, 3*N+4:4*N+4)  = - diag(df3_dx(4,:));
Jac(2*N+4:3*N+4, 4*N+5:5*N+5)  = - diag(df3_dx(5,:));

%Jacobian of initial states
Jac(3*N+5, 1)       = 1;
Jac(3*N+6, N+2)     = 1;
Jac(3*N+7, 2*N+3)   = 1;
Jac(3*N+8, 3*N+4)   = 1;
Jac(3*N+9, 4*N+5)   = 1;

%Jacobian of terminal constraints
Jac(3*N+10, 1)     = 1;
Jac(3*N+11, N+2)   = 1;
Jac(3*N+12, 2*N+3) = 1;

% Jac(3*N+13, 3*N+4) = 1;
% Jac(3*N+14, 4*N+5) = 1;


% Jac=[];
