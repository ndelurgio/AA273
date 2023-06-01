function [mu,cov] = ukf(mu_prev,cov_prev, y, M, J, dt, Q, R)

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
y_predict = zeros(m,2*n+1);
for i = 1:(2*n+1)
    y_predict(:,i) = getSensors(x(:,i),zeros(3,3),zeros(4,4));
end
Ey = 0;
for i = 1:(2*n+1)
    Ey = Ey + w(1,i)*y_predict(:,i);
end
cov_y = zeros(m,m);
for i = 1:(2*n+1)
    cov_y = cov_y + w(1,i)*(y_predict(:,i)-Ey)*(y_predict(:,i)-Ey)';
end
cov_y = cov_y + R;

cov_xy = zeros(n,m);
for i = 1:(2*n+1)
    cov_xy = cov_xy + w(1,i)*(x(:,i)-mu)*(y_predict(:,i)-Ey)';
end

mu = mu + cov_xy*inv(cov_y)*(y-Ey);
cov = cov - cov_xy*inv(cov_y)*cov_xy';

end

