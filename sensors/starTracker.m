function q_meas = starTracker(q,R)
%STARTRACKER Summary of this function goes here
%   Detailed explanation goes here
noise = mvnrnd([0;0;0;0],R)';
q_meas = q + noise;
q_meas = q_meas/norm(q_meas);
end

