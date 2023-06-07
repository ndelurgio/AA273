function [mu,cov] = mekf_err_ang(mu_prev, cov_prev, y, M, J, dt, Q, R)

% Error angle covariance
% Q_er = 0.001*eye(3);
% Predict
mu = zeros(9,1);

%q_ref = state_quat(1:4);
%q_ref_mat = getQmat(q_ref);
%state vector with  error representation of quat
mu(1:6,1) = attitudeDynamicsErr(dt,mu_prev(1:6,1),M,J);
mu(7:9,1) = gyroBiasEuler(mu_prev(7:9,1),[0;0;0],dt);

% q_vec = mu(1:3);
% y_qvec = y(1:3);
% y_wvec = y(5:7);
% w_meas = y_wvec - mu(8:10);
A = jacobianest(@(x) [attitudeDynamicsErr(dt, x(1:6),M,J); gyroBiasEuler(x(7:9),zeros(3,1),dt)], mu_prev);
cov = A*cov_prev*A' + Q;
% Kalman Gain

%C = jacobianest(@(x) quat_err(getSensors(x,R(5:7,5:7),R(1:4,1:4)),q_ref_mat),mu); %6*10
C = jacobianest(@(x) getSensorsErr(x,R(4:6,4:6),R(1:3,1:3)),mu); %6*9
%C_err = [eye(7) zeros(7,3)]; %7*9

K = cov*C'*inv(C*cov*C'+R);


% Expected Measurement
Ey = getSensorsErr(mu,zeros(3,3),zeros(3,3));
% True error measurement given

mu = mu + K*(y - Ey);
%dv = mu(1:3,1);
% Reset
%q = 1/sqrt(1 + norm(dv/2)^2) * q_mult([dv/2;1],q_ref);

% Other parameters
cov = cov - K*C*cov;
%mu = [q; mu_err(4:end)];

end

