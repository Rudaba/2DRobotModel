function runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar)

global M Nx refTraj t0 tf m n N xNav t_sort

A = [];
iAfun = [];
jAvar = [];

if modelNumber == 1
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunDirectShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    %Extract inputs and states
    dtx = (tf - t0) / (N);
    t   = [t0:dtx:tf]';
    
    for k = 1:m
        u(:,k) = x((k-1)*(N+1)+1:k*(N+1));
    end
    
    [xNav,tReal,yReal] = simulateRobotRK(xNav,t,u,0.01,t0,tf);
    
    %Plot results
    figure
    subplot(2,1,1)
    title('Controls, omegaR (top), omegaL (bottom)')
    plot(t,u(:,1),'*-r')
    hold on
    subplot(2,1,2)
    plot(t,u(:,2),'*-b')
    
    figure
    subplot(2,1,1)
    plot(tReal,yReal(:,1),'r')
    title('Integrated States, x (top), y (bottom)')
    hold on
    subplot(2,1,2)
    plot(tReal,yReal(:,2),'b')
    
    figure
    plot(yReal(:,1),yReal(:,2),'g')
    title('x vs y')
    
    
elseif modelNumber == 2
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunMultipleShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    %Extract time, Controls and States
    
    dti = (tf - t0) / (M);
    dtu = dti / (Nx);
    t0j = [t0:dti:tf-dti];
    tfj = [t0+dti:dti:tf];
    
    for j = 1:M
        t{j} = [t0j(j):dtu:tfj(j)];
    end
    
    for j = 1:M
        ystart{j}(:, 1) = x((j-1)*n+1:j*n);
        yend{j}(:, 1) = x(M*n+((j-1)*n+1:j*n));
    end
    
    for j = 1:M
        for k = 1:m
            u{j}(:,k) = x(2*M*n+(j-1)*(Nx+1)+((k-1)*(Nx+1)+1:k*(Nx+1)));
        end
    end
    
    y0 = xNav;
    for j = 1:M
        [xNav,tReal{j},yReal{j}] = simulateRobotRK(y0,t{j},u{j},0.01,t{j}(1),t{j}(end));
        y0 = yReal{j}(end,:);
    end
    
    %Plot results
    figure;
    for j = 1:M
        subplot(2,1,1)
        plot(t{j}(1:Nx),u{j}(1:Nx,1),'*-r'); hold on;
    end
    title('Controls, omegaR (top), omegaL (bottom)')
    
    for j = 1:M
        subplot(2,1,2)
        plot(t{j}(1:Nx),u{j}(1:Nx,2),'*-r'); hold on;
    end
    
    figure;
    for j = 1:M
        subplot(2,1,1)
        plot(tReal{j}(:,1),yReal{j}(:,1)); hold on;
        plot(t{j},ystart{j}(1,1),'*r'); hold on;
        plot(t{j},yend{j}(1,1),'*g')
    end
    title('Integrated States, x (top), y (bottom)')
    
    for j = 1:M
        subplot(2,1,2)
        plot(tReal{j}(:,1),yReal{j}(:,2)); hold on;
        plot(t{j},ystart{j}(2,1),'*r'); hold on;
        plot(t{j},yend{j}(2,1),'*g')
    end
    
    figure
    for j = 1:M
        plot(yReal{j}(:,1),yReal{j}(:,2)); hold on;
    end
    title('x vs y')
    
elseif modelNumber == 3
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunCollocation', A, iAfun, jAvar, iGfun, jGvar);
    
    % Extract states and controls
    for j = 1:n
        y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
    end
    
    for k = 1:m
        u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
    end
        
    dt   = (tf - t0) / (N);
    t    = [t0:dt:tf];
    
    [xNav,tReal,yReal] = simulateRobotRK(xNav,t,u,0.01,t0,tf);
    
    %Plot results
    figure
    subplot(2,1,1)
    title('Controls, omegaR (top), omegaL (bottom)')
    plot(t,u(:,1),'*-r')
    hold on
    subplot(2,1,2)
    plot(t,u(:,2),'*-b')
    
    figure
    subplot(2,1,1)
    plot(tReal,yReal(:,1),'r')
    hold on
    plot(t,y(:,1),'*g')
    title('Integrated States, x (top), y (bottom)')
    hold on
    subplot(2,1,2)
    plot(tReal,yReal(:,2),'b')
    hold on
    plot(t,y(:,2),'*g')
    legend('integrated','optimal')
    
    figure
    plot(yReal(:,1),yReal(:,2),'b')
    hold on
    plot(y(:,1),y(:,2),'*g')
    title('x vs y')
    legend('integrated','optimal')
elseif modelNumber == 4
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunPseudospectral', A, iAfun, jAvar, iGfun, jGvar);
    
    % Extract states and controls
    for j = 1:n
        y(:,j) = x((j-1)*(N+1)+1:j*(N+1));
    end
    
    for k = 1:m
        u(:,k) = x(n*(N+1)+((k-1)*(N+1)+1:k*(N+1)));
    end
        
    t                  = ((tf-t0)/2*t_sort+(tf+t0)/2);
    
    [xNav,tReal,yReal] = simulateRobotRK(xNav,t,u,0.01,t0,tf);
    
    %Plot results
    figure
    subplot(2,1,1)
    title('Controls, omegaR (top), omegaL (bottom)')
    plot(t,u(:,1),'*-r')
    hold on
    subplot(2,1,2)
    plot(t,u(:,2),'*-b')
    
    figure
    subplot(2,1,1)
    plot(tReal,yReal(:,1),'r')
    hold on
    plot(t,y(:,1),'*g')
    title('Integrated States, x (top), y (bottom)')
    hold on
    subplot(2,1,2)
    plot(tReal,yReal(:,2),'b')
    hold on
    plot(t,y(:,2),'*g')
    legend('integrated','optimal')
    
    figure
    plot(yReal(:,1),yReal(:,2),'b')
    hold on
    plot(y(:,1),y(:,2),'*g')
    title('x vs y')
    legend('integrated','optimal')
    
elseif modelNumber == 5
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunLMPCSNOPT', A, iAfun, jAvar, iGfun, jGvar);
    
    [F,Jac,tout,yout,uout] = snoptuserfunLMPCSNOPT(x);
    
    xRef                      = interp1(refTraj(:,1),refTraj(:,2:end),tout);
    
    for i = 1:length(tout)
        [omegaR0,omegaL0]   = calcFeedforward(xRef(i,:));
        
        u(i,1)    = uout(i,1) + omegaR0;
        u(i,2)    = uout(i,2) + omegaL0;
        
        y(i,:)    = xRef(i,1:3) + yout(i,:);
    end
    
    xRef    = interp1(refTraj(:,1),refTraj(:,2:end),tout)';
    
    figure;
    plot(y(:,1),y(:,2),'-b');
    hold on
    plot(xRef(1,:),xRef(2,:),'-r');
    legend('actual','ref')
    
    figure;
    plot(tout,u(:,1),'-b');
    hold on
    plot(tout,u(:,2),'-r');
    
end


