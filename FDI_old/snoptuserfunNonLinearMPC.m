function [F,Jac,tout,yout,uout] = snoptuserfunNonLinearMPC(x)

global N t0 Hp y0 n m refTraj D_sort w t_sort X_Filter

numconstr   = 1+n*(N+1) + (1*n);
F           = zeros(numconstr,1);
y           = zeros(N+1,n);
u           = zeros(N+1,m);
Eqn         = zeros(N+1,n);
tf          = t0 + Hp;
factor      = (tf-t0)/2;

RR          = 2;%X_EKF(4,1);
RL          = 2;%X_EKF(5,1);

% Extract states and controls from x vector
for j = 1:n
    y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
end

for k = 1:m
    u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
end

% if nargout > 3
%     yout = y;
%     uout = u;
% end

t                   = ((tf-t0)/2*t_sort+(tf+t0)/2);

[yDots, v, psiDot, df1_dx, df2_dx, df3_dx]    = stateEquationsNonLinearMPC(y, u, t, RR, RL);

ydots = yDots';

for index=n:-1:1
    Eqn(:,index) = 1./factor.*D_sort*y(:,index) - ydots(:,index);
end

% Assign Objective Function and Constraints to F
xRef                = interp1(refTraj(:,1),refTraj(:,2:end),t)';
% xRef(1:2,:)         = xRef(1:2,:) ./ 10;
[Mayer,Integral,dC_dx]    = integralCost(y',u',v',psiDot',xRef, RR, RL);

F(1) = Mayer + sum(w.*Integral')*factor;

for k = 1:n
    F(1+(k-1).*(N+1)+1:1+k.*(N+1)) = Eqn(:,k);
end

if nargout > 2
    tout = t;
end

% initial conditions
F(1+n.*(N+1)+1) = y(1,1)- y0(1);
F(1+n.*(N+1)+2) = y(1,2)- y0(2);
F(1+n.*(N+1)+3) = y(1,3)- y0(3);

% %Terminal constraints
% F(1+n.*(N+1)+4) = y(end,1) - xRef(1,end);
% F(1+n.*(N+1)+5) = y(end,2) - xRef(2,end);
% F(1+n.*(N+1)+6) = y(end,3) - xRef(3,end);


Jac = zeros(1+n*(N+1)+(1*n),(n+m)*(N+1));

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

% %Jacobian of terminal constraints
% Jac(3*N+8, 1)      = 1;
% Jac(3*N+9, N+2)    = 1;
% Jac(3*N+10, 2*N+3) = 1;


% Jac= [];

