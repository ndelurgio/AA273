function xtp1 = attitudeDynamicsEuler(x, M, J, dt)

xtp1 = x + dt*attitudeDynamics(0,x,M,J);

end

