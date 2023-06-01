close all

tf = 2000;
dt = 10;
tspan = 0:dt:tf;

J = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 2
];
% M = [0.001;0.001;-0.00];
M = zeros(3,1);
Q_gyro = 1E-5*dt*eye(3);
R_gyro = 0.0001*eye(3);
R_starTracker = 0.001*eye(4);
R = [R_starTracker, zeros(4,3); zeros(3,4), R_gyro];
Q_KF = 1E-5*dt*eye(10);

q0 = [0;0;0;1];
% q0 = [0.5;0.5;0.5;0.5];
q0 = q0/norm(q0);
w0 = [0;0.01;0.01];   
% w0 = [0;0.0;0.0];   
b0 = [0;0;0];
x0 = [q0; w0; b0];

x = zeros(length(x0),length(tspan));
x(1:7,1) = [q0;w0];
y = zeros(length([q0;w0]),length(tspan)-1);

mu_ukf = zeros(length(x0),length(tspan)-1);
mu_ukf(:,1) = x0;
cov_ukf = zeros(length(x0),length(x0),length(tspan)-1);
cov_ukf(:,:,1) = 1E-5*dt*eye(10);

mu_mukf = mu_ukf;
cov_mukf = cov_ukf;

mu_ekf4quat = mu_ukf;
cov_ekf4quat = cov_ukf;

for i = 1:(length(x(1,:))-1)
    y(:,i) = getSensors(x(:,i),R_gyro,R_starTracker);
    if i > 1
        [mu_ukf(:,i),cov_ukf(:,:,i)] = ukf(mu_ukf(:,i-1),cov_ukf(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_mukf(:,i),cov_mukf(:,:,i)] = mukf(mu_mukf(:,i-1),cov_mukf(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_ekf4quat(:,i),cov_ekf4quat(:,:,i)] = ekf_4quat(mu_ekf4quat(:,i-1),cov_ekf4quat(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
    end
    x(:,i+1) = propagateState(x(:,i),tspan(i),tspan(i+1),M,J,Q_gyro);
end

q = x(1:4,:);
w = x(5:7,:);
b = x(8:10,:);
mu_q = mu_ukf(1:4,:);
mu_w = mu_ukf(5:7,:);
mu_b = mu_ukf(8:10,:);


plotQuaternion(tspan,q,y,mu_q)
plotAngularVelocity(tspan,w,y,mu_w)
plotGyroBias(tspan,b,mu_b)

plotQuaternionError(tspan,x,mu_ekf4quat,mu_ukf,mu_mukf)
plotAngVelError(tspan,x,mu_ekf4quat,mu_ukf,mu_mukf)
