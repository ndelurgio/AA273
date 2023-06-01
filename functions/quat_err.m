function [Ey] = quat_err(Ey,q_ref_mat)
%QUAT_ERR Summary of this function goes here
Ey_q = Ey(1:4,1);

Ey_dq = inv(q_ref_mat)*Ey_q;
Ey_dv = 2*Ey_dq(1:3,1);

Ey = [Ey_dv; Ey(5:7,1)];
end

