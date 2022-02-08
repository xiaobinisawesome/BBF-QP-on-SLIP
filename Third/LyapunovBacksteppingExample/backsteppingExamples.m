
%% initialize
k1 = 1; k2 = 1; x0 = 1; u0 = 1; Tf = 5;
%% dynamics
%dx = x^3 + u;  du = tau;
%% recurisive linearizing
%u* = -x^3 -k1x %tau_rl = -k2*(u + k1*x);
ustar = @(x) -x.^3 - k1.*x;
tau_rl = @(x,u) -k2*(u - ustar(x)); 
dyn1 = @(X) [X(1)^3 + X(2);  tau_rl(X(1),X(2))];
[T, X] = ode45(@(t,X)dyn1(X), [0, Tf], [x0;u0]);%% simulate 1
xsol = X(:,1);  usol = X(:,2);
tauSol = tau_rl(xsol, usol);
figure, 
subplot(1,3,1), plot(T, xsol); title('x'); 
subplot(1,3,2), plot(T, usol); title('u'); 
subplot(1,3,3), plot(T, tauSol); title('tau'); 
%% backstepping
%du* = -3*x^2*dx - k1*dx; %V = x^2 + 0.5*(u - u*)^2 ;
%tau = -2*x -k2*(u-u*) -(3*x^2 +k1)*dx;
tau_backstepping = @(x,u) -2*x-k2*(u+x.^3+k1*x)-(3*x.^2 +k1).*(x.^3 +u);
%% simulate
dyn2 = @(X) [X(1)^3 + X(2);
    tau_backstepping(X(1),X(2))];
[T, X] = ode45(@(t , X) dyn2(X), [0, Tf], [x0;u0]);%% simulate 2
xsol = X(:,1);  usol = X(:,2);
tauSol = tau_backstepping(xsol, usol);
figure, 
subplot(1,3,1), plot(T, xsol); title('x');
subplot(1,3,2), plot(T, usol); title('u');
subplot(1,3,3), plot(T, tauSol); title('tau'); 
