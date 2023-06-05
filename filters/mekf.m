function [mu,cov] = mekf(mu_prev, cov_prev, y, M, J, dt, Q, R)

% Error angle covariance
% Q_er = 0.001*eye(3);
% Predict
mu = zeros(10,1);
mu(1:7,1) = attitudeDynamics4rk(mu_prev(1:7,1),M,J,dt);
mu(1:4,1) = mu(1:4,1)/norm(mu(1:4,1));
mu(8:10,1) = gyroBiasEuler(mu_prev(8:10,1),[0;0;0],dt);
q_ref = mu(1:4);
q_ref_mat = getQmat(q_ref);
mu_err = [0;0;0;mu(5:10)];
% q_vec = mu(1:3);
% y_qvec = y(1:3);
% y_wvec = y(5:7);
% w_meas = y_wvec - mu(8:10);
A = jacobianest(@(x) [attitudeDynamics4rk(x(1:7),M,J,dt); gyroBiasEuler(x(8:10),zeros(3,1),dt)], mu_prev);
cov = A*cov_prev*A' + Q;
% Kalman Gain

%C = jacobianest(@(x) quat_err(getSensors(x,R(5:7,5:7),R(1:4,1:4)),q_ref_mat),mu); %6*10
C = jacobianest(@(x) getSensors(x,zeros(3,3),zeros(4,4)),mu); %7*10
C_err = [eye(6) zeros(6,3)]; %6*9

K = cov*C'*inv(C*cov*C'+R);
K_err = cov([1:3, 5:end],[1:3, 5:end])*C_err'*inv(C_err*cov([1:3, 5:end],[1:3, 5:end])*C_err'+R([1:3,5:end],[1:3,5:end]));

% Expected Measurement
Ey = getSensors(mu,zeros(3,3),zeros(4,4));
Ey = quat_err(Ey,q_ref_mat);
% True error measurement
y = quat_err(y,q_ref_mat);

mu_err = mu_err + K_err*(y - Ey);
dv = mu_err(1:3,1);
% Reset
q = 1/sqrt(1 + norm(dv/2)^2) * q_mult([dv/2;1],q_ref);

% Other parameters
cov = cov - K*C*cov;
mu = [q; mu_err(4:end)];

end

