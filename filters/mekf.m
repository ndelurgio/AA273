function [mu,cov] = mekf(mu_prev, cov_prev, y, M, J, dt, Q, R)

% Error angle covariance
% Q_er = 0.001*eye(3);
% Predict
mu = zeros(10,1);
mu(1:7,1) = attitudeDynamicsEuler(mu_prev(1:7,1),M,J,dt);
mu(1:4,1) = mu(1:4,1)/norm(mu(1:4,1));
mu(8:10,1) = gyroBiasEuler(mu_prev(8:10,1),[0;0;0],dt);
q = mu(1:4);
% q_vec = mu(1:3);
y_qvec = y(1:3);
y_wvec = y(5:7);
% w_meas = y_wvec - mu(8:10);
A = jacobianest(@(x) [attitudeDynamicsEuler(x(1:7),M,J,dt); gyroBiasEuler(x(8:10),zeros(3,1),dt)], mu_prev);
cov = A*cov_prev*A' + Q;
% Kalman Gain

C = jacobianest(@(x) getSensors(x,R(5:7,5:7),R(1:4,1:4)),mu);
K = cov*C'*inv(C*cov*C'+R);
E = [
    q(4), -q(3), q(2);
    q(3), q(4), -q(1);
   -q(2), q(1), q(4);
   -q(1),-q(2),-q(3);
];
H = 0.5*C(1:3,1:4)*E;
% Expected Measurement
Ey = getSensors(mu,zeros(3,3),zeros(4,4));
% dv = K(1:3,1:3)*(y_qvec - Ey(1:3));
dv = K(5:7,5:7)*(y_wvec - Ey(5:7));
dv = (eye(3) - K(1:3,1:3)*H)*dv + K(1:3,1:3)*(y_qvec/norm(y_qvec) - Ey(1:3)/norm(Ey(1:3)));
% Reset
q = 1/sqrt(1 + norm(dv/2)^2) * q_mult([dv/2;1],q);

% Other parameters
mu = mu + K*(y-Ey);
cov = cov - K*C*cov;
mu(1:4) = q;


end

