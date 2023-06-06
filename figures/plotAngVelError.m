function plotAngVelError(t,x,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)

w_ekf4quat = mu_ekf4quat(5:7,:);
w_ekf3quat = mu_ekf3quat(5:7,:);
w_mekf = mu_mekf(5:7,:);
w_ukf = mu_ukf(5:7,:);
w_mukf = mu_mukf(5:7,:);
w_true = x(5:7,1:end-1);

ekf4quat_err = rad2deg(vecnorm(w_ekf4quat-w_true,2,1));
ekf3quat_err = rad2deg(vecnorm(w_ekf3quat-w_true,2,1));
mekf_err = rad2deg(vecnorm(w_mekf-w_true,2,1));
ukf_err = rad2deg(vecnorm(w_ukf-w_true,2,1));
mukf_err = rad2deg(vecnorm(w_mukf-w_true,2,1));

figure
hold on;
grid on;
k = 25;
plot(t(1:end-1),movmean(ekf4quat_err,floor(length(ekf4quat_err)/k)),LineWidth=2)
plot(t(1:end-1),movmean(ekf3quat_err,floor(length(ekf3quat_err)/k)),LineWidth=2)
plot(t(1:end-1),movmean(mekf_err,floor(length(mekf_err)/k)),LineWidth=2)
plot(t(1:end-1),movmean(ukf_err,floor(length(ukf_err)/k)),LineWidth=2)
plot(t(1:end-1),movmean(mukf_err,floor(length(mukf_err)/k)),LineWidth=2)
xlabel("Time [s]")
ylabel("\omega Error [deg/s]")
legend(["EKF4","EKF3","MEKF","UKF","MUKF"])


end

