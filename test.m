close all
clc

tf = 750;
dt = 60;
tspan = 0:dt:tf;

J = [
    100, 0, 0;
    0, 100, 0;
    0, 0, 200
];
% M = [0.001;0.001;-0.00];
M = zeros(3,1);
Q_gyro = 1E-8*eye(3);
R_gyro = 1E-7*eye(3);
R_starTracker = 1E-4*eye(4);
R = [R_starTracker, zeros(4,3); zeros(3,4), R_gyro];
Q_KF = 1E-6*dt*eye(10);
Q_KF(8:10,8:10) = 1E-8*dt*eye(3);

Q_KF_MEKF = 1E-6*dt*eye(10);
Q_KF_MEKF(8:10,8:10) = 1E-8*dt*eye(3);

Q_KF_MEKF_err = 1E-4*dt*eye(9);
Q_KF_MEKF_err(7:9,7:9) = 1E-8*dt*eye(3);
R_ERR = [R_starTracker(1:3,1:3), zeros(3,3); zeros(3,3), R_gyro];

q0 = [0;0;0;1];
% q0 = [0.5;0.5;0.5;0.5];
q0 = q0/norm(q0);
w0 = [0;0.0;deg2rad(0.0796)];   
% w0 = [0;0.0;0.0];   
b0 = [0;0;0];
x0 = [q0; w0; b0];

x = zeros(length(x0),length(tspan));
x(1:7,1) = [q0;w0];
y = zeros(length([q0;w0]),length(tspan)-1);

x_err = zeros(9,length(tspan));
x_err(1:6,1) = [0;0;0;w0];
y_err = zeros(6,length(tspan)-1);

mu_ukf = zeros(length(x0),length(tspan)-1);
mu_ukf(:,1) = x0;
cov_ukf = zeros(length(x0),length(x0),length(tspan)-1);
cov_ukf(:,:,1) = 1E-5*dt*eye(10);

mu_mukf = mu_ukf;
cov_mukf = cov_ukf;

mu_ekf4quat = mu_ukf;
cov_ekf4quat = cov_ukf;

mu_ekf3quat = mu_ukf;
cov_ekf3quat = cov_ukf;

mu_mekf = mu_ukf;
cov_mekf = cov_ukf;

mu_mekf_err = zeros(9,length(tspan)-1);
mu_mekf_err(:,1) = [0;0;0;w0;b0];
cov_mekf_err = zeros(9,9,length(tspan)-1);
cov_mekf_err(:,:,1) = 1E-5*dt*eye(9);
q_ref = zeros(4,length(tspan)-1);
q_ref(:,1) = q0;


for i = 1:(length(x(1,:))-1)
    disp(i)
    t = tspan(i);
    if t < 50
        M = [0.001;0;0];
    elseif t > t(end)-50
        M = [-0.001;0;0];
    else
        M = [0;0;0];
    end
    

    y(:,i) = getSensors(x(:,i),R_gyro,R_starTracker);
    
    if i > 1
        [mu_ukf(:,i),cov_ukf(:,:,i)] = ukf(mu_ukf(:,i-1),cov_ukf(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_mukf(:,i),cov_mukf(:,:,i)] = mukf(mu_mukf(:,i-1),cov_mukf(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_ekf4quat(:,i),cov_ekf4quat(:,:,i)] = ekf_4quat(mu_ekf4quat(:,i-1),cov_ekf4quat(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_ekf3quat(:,i),cov_ekf3quat(:,:,i)] = ekf(mu_ekf3quat(:,i-1),cov_ekf3quat(:,:,i-1),y(:,i),M,J,dt,Q_KF,R);
        [mu_mekf(:,i),cov_mekf(:,:,i)] = mekf(mu_mekf(:,i-1),cov_mekf(:,:,i-1),y(:,i),M,J,dt,Q_KF_MEKF,R);
        
        % Err MEKF
        % Update Error State
        q_curr  = x(1:4,i);
        err = mu_mekf_err(1:3,i-1);
        % disp(err')
        dq  = 1/sqrt(1+norm(err/2)^2) * q_mult([err/2;1],q_ref(:,i-1));
        q_ref(:,i)   = q_mult(dq,q_ref(:,i-1));
        q_ref(:,i) = q_ref(:,i)/norm(q_ref(:,i));
        disp(q_ref(:,i)')
        q_ref_mat = getQmat(q_ref(:,i));
        dq = inv(q_ref_mat)*q_curr;
        dq = dq/norm(dq);
        err = 2*dq(1:3,1);
        x_err(:,i) = [err;x(5:end,i)];
        y_err(:,i) = getSensorsErr(x_err(:,i), R_gyro, R_starTracker(1:3,1:3));
        mu_mekf_err(1:3,i-1) = [0;0;0];

        [mu_mekf_err(:,i),cov_mekf_err(:,:,i)] = mekf_err_ang(mu_mekf_err(:,i-1),cov_mekf_err(:,:,i-1),y_err(:,i),M,J,dt,Q_KF_MEKF_err,R_ERR);
        % disp(mu_mekf_err(1:3,i)')
    end
    x(:,i+1) = propagateState(x(:,i),tspan(i),tspan(i+1),M,J,Q_gyro);
end

q = x(1:4,:);
w = x(5:7,:);
b = x(8:10,:);
mu_q = mu_mekf(1:4,:);
mu_w = mu_mekf(5:7,:);
mu_b = mu_mekf(8:10,:);


plotQuaternion(tspan,x,y,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)
plotAngularVelocity(tspan,x,y,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)
% plotGyroBias(tspan,b,mu_b)

plotQuaternionError(tspan,x,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)
plotAngVelError(tspan,x,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)
plotBiasTracking(tspan,x,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)

