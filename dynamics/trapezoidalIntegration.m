function x = trapezoidalIntegration(tspan,x0, M, J)
%TRAPEZOIDALINTEGRATION Summary of this function goes here
%   Detailed explanation goes here

x = zeros(length(x0),length(tspan));
x(:,1) = x0;
for i = 1:(length(tspan)-1)
    % first, normalize q
    x(1:4,i) = x(1:4,i)/norm(x(1:4,i));
    dt = tspan(i+1)-tspan(i);
    fx = dt*attitudeDynamics(x(:,i), M, J);
    xp1 = x(:,i) + dt*attitudeDynamics(x(:,i), M, J);
    fxp1 = dt*attitudeDynamics(xp1, M, J);
    x(:,i+1) = x(:,i) + 1/2*(fx+fxp1);
end

end

