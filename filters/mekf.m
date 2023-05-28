function [outputArg1,outputArg2] = mekf(mu_prev, y, M, J, dt, Q_w)

% Error angle covariance
Q_er = 0.001*eye(3);
% Predict
mu = attitudeDynamicsEuler(mu_prev,M,J,dt);



end

