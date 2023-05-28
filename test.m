close all
J = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 2
];
M = [0.01;0.01;-0.01];
Q_gyro = 0.0001*eye(3);
R_gyro = 0.001*eye(3);
R_starTracker = 0.001*eye(4);

tf = 100;
dt = 0.1;
tspan = 0:dt:tf;

q0 = [0;0;0;1];
w0 = [0;0;0];   
b0 = [0;0;0];
x0 = [q0; w0; b0];

x = zeros(length(x0),length(tspan));
x(1:7,1) = [q0;w0];
y = zeros(length([q0;w0]),length(tspan)-1);

for i = 1:(length(x(1,:))-1)
    y(1:4,i) = starTracker(x(1:4,i),R_starTracker);
    y(5:7,i) = gyro(x(5:7,i), x(8:10,i), R_gyro);
    x(:,i+1) = propagateState(x(:,i),tspan(i),tspan(i+1),M,J,Q_gyro);
end

q = x(1:4,:);
w = x(5:7,:);
b = x(8:10,:);

plotQuaternion(tspan,q,y)
plotAngularVelocity(tspan,w,y)
plotGyroBias(tspan,b)

jacobianest(@(x) attitudeDynamicsEuler(x,M,J,dt),[q0;w0])