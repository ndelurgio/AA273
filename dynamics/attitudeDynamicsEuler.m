function xtp1 = attitudeDynamicsEuler(x, M, J, dt)

xtp1 = x + dt*attitudeDynamics(x,M,J,0);
xtp1(1:4,1) = xtp1(1:4,1)/norm(xtp1(1:4,1));

end

