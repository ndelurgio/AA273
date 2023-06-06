function plotBiasTracking(t,x,mu_ekf4quat,mu_ekf3quat,mu_ukf,mu_mukf)

figure()
subplot 311
hold on;
plot(t(1:end),rad2deg(x(8,:)),Linewidth=2)
plot(t(1:end-1),rad2deg(mu_ekf4quat(8,:)))
plot(t(1:end-1),rad2deg(mu_ekf3quat(8,:)))
plot(t(1:end-1),rad2deg(mu_ukf(8,:)))
plot(t(1:end-1),rad2deg(mu_mukf(8,:)))
ylabel("b_x [deg/s]")
legend(["True","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 312
hold on;
plot(t(1:end),rad2deg(x(9,:)),Linewidth=2)
plot(t(1:end-1),rad2deg(mu_ekf4quat(9,:)))
plot(t(1:end-1),rad2deg(mu_ekf3quat(9,:)))
plot(t(1:end-1),rad2deg(mu_ukf(9,:)))
plot(t(1:end-1),rad2deg(mu_mukf(9,:)))
ylabel("b_y [deg/s]")
legend(["True","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 313
hold on;
plot(t(1:end),rad2deg(x(10,:)),Linewidth=2)
plot(t(1:end-1),rad2deg(mu_ekf4quat(10,:)))
plot(t(1:end-1),rad2deg(mu_ekf3quat(10,:)))
plot(t(1:end-1),rad2deg(mu_ukf(10,:)))
plot(t(1:end-1),rad2deg(mu_mukf(10,:)))
ylabel("b_z [deg/s]")
legend(["True","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

end

