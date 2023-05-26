J = eye(3);
M = [0.01;0.01;-0.01];

tf = 100;
dt = 0.1;
tspan = 0:dt:tf;

q0 = [0;0;0;1];
w0 = [0;0;0];
x0 = [q0; w0];

% [t,x] = ode45(@(t,x) attitudeDynamicsWrapper(t,x,M,J),tspan,x0);
x = trapezoidalIntegration(tspan,x0,M,J);
q = x(1:4,:);
w = x(5:7,:);

plotQuaternion(tspan,q)
plotAngularVelocity(tspan,w)