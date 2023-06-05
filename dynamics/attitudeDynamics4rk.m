function xtp1 = attitudeDynamics4rk(x, M, J, dt)

k1 = attitudeDynamics(0,x,M,J);
k2 = attitudeDynamics(0,x+(dt/2)*k1,M,J);
k3 = attitudeDynamics(0,x+(dt/2)*k2,M,J);
k4 = attitudeDynamics(0,x+(dt)*k3,M,J);

xtp1 = x + (dt/6)*(k1+2*k2+2*k3+k4);
xtp1(1:4,1) = xtp1(1:4,1)/norm(xtp1(1:4,1));

end

