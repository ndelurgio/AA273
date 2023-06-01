function [mu,cov] = ekf(mu_prev, cov_prev, y, M, J, dt, Q, R)

            % PREDICT 
mu = zeros(10,1);

% mu = f(x,u)
mu(1:7,1) = attitudeDynamicsEuler(mu_prev(1:7,1),M,J,dt);
mu(1:4,1) = mu(1:4,1)/norm(mu(1:4,1));
mu(8:10,1) = gyroBiasEuler(mu_prev(8:10,1),[0;0;0],dt);
A = jacobianest(@(x) [attitudeDynamicsEuler(x(1:7),M,J,dt); gyroBiasEuler(x(8:10),zeros(3,1),dt)], mu_prev); % compute Jacobian (linearize dynamics about mu_prev)
cov = A*cov_prev*A' + Q;

            % UPDATE
C = jacobianest(@(x) getSensors(x,R(5:7,5:7),R(1:4,1:4)),mu); % compute Jacobian
K = cov*C'*inv(C*cov*C'+R); % Kalman Gain
Ey = getSensors(mu,zeros(3,3),zeros(4,4)); % Expected Measurement, g(x,u) 
meas_err = y - Ey; % Measurement Error

mu = mu + K*meas_err; % Updating State Estimate (mean)
% mu(1:4,1) = mu(1:4,1)/norm(mu(1:4,1)); % normalize quaternion
mu(4,1) = sqrt(1 - mu(1,1)^2 - mu(2,1)^2 - mu(3,1)^2); % 3-parameter quaternion representation to preserve unit norm 
% a = 0.005; 
% if mu(4,1) - a <= 0 | mu(4,1) + a >= 1/2
%     % do something (switch with q1, q2, or q3 to avoid chattering)
% end
cov = cov - K*C*cov; % Updating Confidence of State Estimate (covariance)

end
