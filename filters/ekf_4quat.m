function [mu,cov] = ekf_4quat(mu_prev, cov_prev, y, M, J, dt, Q, R)

% Error angle covariance
% Q_er = 0.001*eye(3);
% Predict
mu = zeros(10,1);
mu(1:7,1) = attitudeDynamics4rk(mu_prev(1:7,1),M,J,dt);
mu(8:10,1) = gyroBiasEuler(mu_prev(8:10,1),[0;0;0],dt);
A = jacobianest(@(x) [attitudeDynamics4rk(x(1:7),M,J,dt); gyroBiasEuler(x(8:10),zeros(3,1),dt)], mu_prev);
cov = A*cov_prev*A' + Q;
% Kalman Gain
C = jacobianest(@(x) getSensors(x,R(5:7,5:7),R(1:4,1:4)),mu);
K = cov*C'*inv(C*cov*C'+R);
% Expected Measurement
Ey = getSensors(mu,zeros(3,3),zeros(4,4));
% Update
mu = mu + K*(y-Ey);
cov = cov - K*C*cov;
% mu(1:4) = q;


end

