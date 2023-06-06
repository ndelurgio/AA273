function plotAngularVelocity(t,x,y,mu_ekf4quat,mu_ekf3quat,mu_ukf,mu_mukf)

t = t(1:end-1);
w = x(5:7,1:end-1);

figure()
subplot 311
hold on;
plot(t,rad2deg(w(1,:)),Linewidth=2)
plot(t,rad2deg(y(5,:)))
plot(t,rad2deg(mu_ekf4quat(5,:)))
plot(t,rad2deg(mu_ekf3quat(5,:)))
plot(t,rad2deg(mu_ukf(5,:)))
plot(t,rad2deg(mu_mukf(5,:)))
ylabel("\omega_x [deg/s]")
legend(["True","IMU Measurement","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 312
hold on;
plot(t,rad2deg(w(2,:)),Linewidth=2)
plot(t,rad2deg(y(6,:)))
plot(t,rad2deg(mu_ekf4quat(6,:)))
plot(t,rad2deg(mu_ekf3quat(6,:)))
plot(t,rad2deg(mu_ukf(6,:)))
plot(t,rad2deg(mu_mukf(6,:)))
ylabel("\omega_y [deg/s]")
legend(["True","IMU Measurement","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 313
hold on;
plot(t,rad2deg(w(3,:)),Linewidth=2)
plot(t,rad2deg(y(7,:)))
plot(t,rad2deg(mu_ekf4quat(7,:)))
plot(t,rad2deg(mu_ekf3quat(7,:)))
plot(t,rad2deg(mu_ukf(7,:)))
plot(t,rad2deg(mu_mukf(7,:)))
ylabel("\omega_z [deg/s]")
xlabel("Time [s]")
legend(["True","IMU Measurement","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

end

