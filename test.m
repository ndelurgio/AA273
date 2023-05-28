close all
J = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 2
];
% M = [0.01;0.01;-0.01];
M = zeros(3,1);
Q_gyro = 1E-5*dt*dt*eye(3);
R_gyro = 0.0001*eye(3);
R_starTracker = 0.001*eye(4);
R = [R_starTracker, zeros(4,3); zeros(3,4), R_gyro];
Q_KF = 1E-5*dt*eye(10);

tf = 2000;
dt = 1;
tspan = 0:dt:tf;

% q0 = [0;0;0;1];
q0 = [0.5;0.5;0.5;0.5];
q0 = q0/norm(q0);
w0 = [0;0.01;0.01];   
% w0 = [0;0.0;0.0];   
b0 = [0;0;0];
x0 = [q0; w0; b0];

x = zeros(length(x0),length(tspan));
x(1:7,1) = [q0;w0];
y = zeros(length([q0;w0]),length(tspan)-1);
mu = zeros(length(x0),length(tspan)-1);
mu(:,1) = x0;
cov = zeros(length(x0),length(x0),length(tspan)-1);
cov(:,:,1) = 1E-5*dt*eye(10);

for i = 1:(length(x(1,:))-1)
    y(:,i) = getSensors(x(:,i),R_gyro,R_starTracker);
    if i > 1
        [mu(:,i),cov(:,:,i)] = mekf(mu(:,i-1),cov(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
    end
    x(:,i+1) = propagateState(x(:,i),tspan(i),tspan(i+1),M,J,Q_gyro);
end

q = x(1:4,:);
w = x(5:7,:);
b = x(8:10,:);
mu_q = mu(1:4,:);
mu_w = mu(5:7,:);
mu_b = mu(8:10,:);


plotQuaternion(tspan,q,y,mu_q)
plotAngularVelocity(tspan,w,y,mu_w)
plotGyroBias(tspan,b,mu_b)
