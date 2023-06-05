function xtp1 = attitudeDynamicsEuler3quat(x, M, J, dt)

xtp1 = x + dt*attitudeDynamics(0,x,M,J);
q = xtp1(1:4,1);
[~,i] = max(q);
q_rest = [q(1:i-1,1);q(i+1:end,1)];
q(i,1) = sqrt(1-norm(q_rest)^2);
xtp1(1:4,1) = q;
% if xtp1(1,1)^2 + xtp1(2,1)^2 + xtp1(3,1)^2 > 1
%     xtp1(1:4,1) = xtp1(1:4,1)/norm(xtp1(1:4,1));
% else
%     xtp1(4,1) = sqrt( 1 - xtp1(1,1)^2 - xtp1(2,1)^2 - xtp1(3,1)^2);
% end

end

