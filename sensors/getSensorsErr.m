function y = getSensorsErr(x,R_gyro,R_starTracker)

y_gyro = gyro(x(4:6),x(7:9),R_gyro);
noise = mvnrnd([0;0;0],R_starTracker)';
y_err_ang = x(1:3) + noise;
y = [y_err_ang;y_gyro];

end


