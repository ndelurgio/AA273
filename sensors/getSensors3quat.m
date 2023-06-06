function y = getSensors3quat(x,R_gyro,R_starTracker)

y_gyro = gyro(x(5:7),x(8:10),R_gyro);
y_starTracker = starTracker(x(1:4),R_starTracker);

q = y_starTracker(1:4,1);
[~,i] = max(q);
q_rest = [q(1:i-1,1);q(i+1:end,1)];
q(i,1) = sqrt(1-norm(q_rest)^2);
y_starTracker(1:4,1) = q;


% if y_starTracker(1,1)^2 + y_starTracker(2,1)^2 + y_starTracker(3,1)^2 > 1
%     y_starTracker(1:4,1) = y_starTracker(1:4,1)/norm(y_starTracker(1:4,1));
% else
%     y_starTracker(4,1) = sqrt( 1 - y_starTracker(1,1)^2 - y_starTracker(2,1)^2 - y_starTracker(3,1)^2);
% end
% y_starTracker(4,1) = sqrt(1-y_starTracker(3,1)^2 - y_starTracker(2,1)^2 - y_starTracker(1,1)^2);

y = [y_starTracker;y_gyro];

end

