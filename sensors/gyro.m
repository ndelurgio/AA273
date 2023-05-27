function w_meas = gyro(w_true,b_true,R)

noise = mvnrnd([0;0;0],R)';
w_meas = w_true + b_true + noise;

end