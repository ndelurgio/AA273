function xdot = attitudeDynamics(x, M, J)
% Input: current state, q 4x1 and w 3x1; Torque T 3x1, inertia tensor J 3x3
% Output: time derivative qdot 4x1, wdot 3x1
% First, normalize quaternion:
q = x(1:4,1);
w = x(5:7,1);
q = q / norm(q);

qdot = 1/2 * skew(w) * q;
wdot = J \ (M - cross(w,J*w));

xdot = [qdot; wdot];
end