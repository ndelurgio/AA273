close all
clc; clear;

tf = 2000;
dt = 10;
tspan = 0:dt:tf;

J = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 2
];
Q_gyro = 1E-5*dt*eye(3);
R_gyro = 0.0001*eye(3);
R_starTracker = 0.001*eye(4);
R = [R_starTracker, zeros(4,3); zeros(3,4), R_gyro];
Q_KF = 1E-5*dt*eye(10);

q0 = [0;0;0;1];
% q0 = [0.5;0.5;0.5;0.5];
q0 = q0/norm(q0);
w0 = [0;0.0;0.00796];   
% w0 = [0;0.0;0.0];   
b0 = [0;0;0];
x0 = [q0; w0; b0];
cov0 = 1E-5*dt*eye(10);

x = zeros(length(x0),length(tspan));
x(1:7,1) = [q0;w0];
y = zeros(length([q0;w0]),length(tspan)-1);

% Simulate True Trajectory 
for i = 1:(length(x(1,:))-1)
    t = tspan(i);
    if t < 50
        M = [0.0001;0;0];
    elseif t > t(end)-50
        M = [-0.0001;0;0];
    else
        M = [0;0;0];
    end
    y(:,i) = getSensors(x(:,i),R_gyro,R_starTracker);
    x(:,i+1) = propagateState(x(:,i),tspan(i),tspan(i+1),M,J,Q_gyro);
end

% Measure mean computation time over some number of runs 
runs = 10;
t_ukf = computationTime(runs,tspan,@ukf,x0,cov0,y,J,dt,Q_KF,R)
% t_mukf = computationTime(runs,tspan,@mukf,x0,cov0,y,J,dt,Q_KF,R)
% t_ekf4 = computationTime(runs,tspan,@ekf_4quat,x0,cov0,y,J,dt,Q_KF,R)
% t_ekf3 = computationTime(runs,tspan,@ekf,x0,cov0,y,J,dt,Q_KF,R)
% t_mekf = computationTime(runs,tspan,@mekf,x0,cov0,y,J,dt,Q_KF,R)
