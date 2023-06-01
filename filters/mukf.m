function [mu,cov] = mukf(mu_prev,cov_prev, y, M, J, dt, Q, R)

n = length(mu_prev);
m = length(y);
[x,w] = UT(mu_prev,cov_prev);
% Predict
for i = 1:(2*n+1)
    x(1:7,i) = attitudeDynamicsEuler(x(1:7,i),M,J,dt);
    x(8:10,i) = gyroBiasEuler(x(8:10,i),[0;0;0],dt);
end
[mu,cov] = UTinv(x,w);
cov = cov + Q;
% Update
[x,w] = UT(mu,cov);
q_ref = mu(1:4,1);
q_ref_mat = getQmat(q_ref);
y_predict = zeros(m-1,2*n+1);
for i = 1:(2*n+1)
    Ey = getSensors(x(:,i),zeros(3,3),zeros(4,4));
    % Convert q measurement to a dv measurement
    Ey_q = Ey(1:4,1);
    Ey_dq = inv(q_ref_mat)*Ey_q;
    Ey_dv = 2*Ey_dq(1:3,1);
    % Store dv measurement in for y measurement
    y_predict(:,i) = [Ey_dv; Ey(5:7,1)];
end
Ey = 0;
for i = 1:(2*n+1)
    Ey = Ey + w(1,i)*y_predict(:,i);
end
cov_y = zeros(m-1,m-1);
for i = 1:(2*n+1)
    cov_y = cov_y + w(1,i)*(y_predict(:,i)-Ey)*(y_predict(:,i)-Ey)';
end
cov_y = cov_y + R(2:end,2:end); %% TEMP

mu_err = [0;0;0;mu(5:end)];
% Cross-cov using error angles
cov_xy_err = zeros(n-1,m-1);
for i = 1:(2*n+1)
    xi = x(:,i);
    % Convert q state into dv state
    dqi = inv(q_ref_mat)*xi(1:4,1);
    dvi = 2*dqi(1:3,1);
    xi = [dvi; xi(5:end,1)];
    % Sum for err state covariance
    cov_xy_err = cov_xy_err + w(1,i)*(xi-mu_err)*(y_predict(:,i)-Ey)';
end

% True error measurement
y_q = y(1:4,1);
y_dq = inv(q_ref_mat)*y_q;
y_dv = 2*y_dq(1:3,1);
y = [y_dv; y(5:7,1)];

% Update state and dv
mu_err = mu_err + cov_xy_err*inv(cov_y)*(y-Ey);
dv = mu_err(1:3,1);

% Update covariance using normal state representation
cov_xy = zeros(n,m-1);
for i = 1:(2*n+1)
    cov_xy = cov_xy + w(1,i)*(x(:,i)-mu)*(y_predict(:,i)-Ey)';
end
cov = cov - cov_xy*inv(cov_y)*cov_xy';

% Update quaternion rigerously using error angles
mu_q = 1/sqrt(1 + norm(dv/2)^2) * q_mult([dv/2;1],q_ref);
mu = [mu_q; mu_err(4:end)];

end

