function y = getSensors(x,R_gyro,R_starTracker)

y_gyro = gyro(x(5:7),x(8:10),R_gyro);
y_starTracker = starTracker(x(1:4),R_starTracker);
y = [y_starTracker;y_gyro];

end

