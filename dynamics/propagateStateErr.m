function [xf] = propagateStateErr(x0,t0,tf,M,J,Q_gyro)
xf = zeros(9,1);
[~,x_temp] = ode45(@(t,x) attitudeDynamicsErr(t,x,M,J), [t0, tf], x0(1:6,1));
% Draw noise seed before ode45 for huge speed-up
random_walk = mvnrnd([0;0;0],Q_gyro)';
[~,b_temp] = ode45(@(t,x) gyroBias(t,x,random_walk), [t0, tf], x0(7:9,1));
xf(1:6,1) = x_temp(end,1:6)';
xf(7:9,1) = b_temp(end,:)';

end