function xtp1 = attitudeDynamicsEuler(x, M, J, dt)

xtp1 = x + dt*attitudeDynamics(0,x,M,J);
xtp1(1:4,1) = xtp1(1:4,1)/norm(xtp1(1:4,1));

end

