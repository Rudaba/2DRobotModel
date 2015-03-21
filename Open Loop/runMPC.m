function runMPC(modelNumber,x,xlow,xupp,Flow,Fupp,iGfun,jGvar)

global M Nx refTraj t0 tf m n N xNav

A = [];
iAfun = [];
jAvar = [];

if modelNumber == 1
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunDirectShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    dtx = (tf - t0) / (N);
    t   = [t0:dtx:tf]';
    
    for k = 1:m
        u(:,k) = x((k-1)*(N+1)+1:k*(N+1));
    end
    
    figure
    subplot(2,1,1)
    title('Controls, omegaR (top), omegaL (bottom)')
    plot(t,u(:,1),'*-r')
    hold on
    subplot(2,1,2)
    plot(t,u(:,2),'*-b')
    
    [xNav,tReal,yReal] = simulateRobotRK(xNav,t,u,0.01,t0,tf);
    
    figure
    subplot(2,1,1)
    plot(tReal,yReal(:,1),'*-r')
    title('Integrated States, x (top), y (bottom)')
    hold on
    subplot(2,1,2)
    plot(tReal,yReal(:,2),'*-b')
    
    figure
    plot(yReal(:,1),yReal(:,2),'*-g')
    title('x vs y')
    
    
elseif modelNumber == 2
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunMultipleShooting', A, iAfun, jAvar, iGfun, jGvar);
    
    [F,Jac,tout,yout,uout] = snoptuserfunMultipleShooting(x);
    
    figure;
    for j = 1:M
        plot(tout{j}(1:Nx),uout{j}(1:Nx)); hold on;
    end
    
    figure;
    for j = 1:M
        plot(tout{j}(1:Nx+1),yout{j}(:,1:Nx+1)); hold on;
    end
    
elseif modelNumber == 3
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunCollocation', A, iAfun, jAvar, iGfun, jGvar);
    
    [F,Jac,tout,yout,uout] = snoptuserfunCollocation(x);
    
    xRef    = interp1(refTraj(:,1),refTraj(:,2:end),tout)';
    
    figure;
    plot(yout(:,1),yout(:,2),'-b');
    hold on
    plot(xRef(1,:),xRef(2,:),'-r');
    legend('actual','ref')
    
    figure;
    plot(tout,uout(:,1),'-b');
    hold on
    plot(tout,uout(:,2),'-r');
    
elseif modelNumber == 4
    
    [x,F,inform,xmul,Fmul] = snopt(x,xlow,xupp,Flow,Fupp,'snoptuserfunPseudospectral', A, iAfun, jAvar, iGfun, jGvar);
    
    [F,Jac,tout,yout,uout] = snoptuserfunPseudospectral(x);
    
    xRef    = interp1(refTraj(:,1),refTraj(:,2:end),tout)';
    
    figure;
    plot(yout(:,1),yout(:,2),'-b');
    hold on
    plot(xRef(1,:),xRef(2,:),'-r');
    legend('actual','ref')
    
    figure;
    plot(tout,uout(:,1),'-b');
    hold on
    plot(tout,uout(:,2),'-r');
    
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


