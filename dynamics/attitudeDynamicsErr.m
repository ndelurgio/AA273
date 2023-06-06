function x_new = attitudeDynamicsErr(dt, x, M, J)
% Input: current state, error angle 3x1 and w 3x1; Torque T 3x1, inertia tensor J 3x3
% Output: error angle 3x1, wdot 3x1

err = x(1:3,1);
w = x(4:6,1);

errdot =  -cross(w ,err);
wdot = J \ (M - cross(w,J*w));

xdot = [errdot; wdot];
x_new = x + dt*xdot; %euler
end

