function plotAngVelError(t,x,mu_ekf4quat,mu_ukf,mu_mukf)

w_ekf4quat = mu_ekf4quat(5:7,:);
% w_ekf3quat = mu_ekf3quat(5:7,:);
w_ukf = mu_ukf(5:7,:);
w_mukf = mu_mukf(5:7,:);
w_true = x(5:7,1:end-1);

ekf4quat_err = vecnorm(w_ekf4quat-w_true,2,1);
% ekf3quat_err = vecnorm(w_ekf3quat-w_true,2,1);
ukf_err = vecnorm(w_ukf-w_true,2,1);
mukf_err = vecnorm(w_mukf-w_true,2,1);

figure
hold on;
plot(t(1:end-1),ekf4quat_err,LineWidth=2)
% plot(t(1:end-1),ekf3quat_err,LineWidth=2)
plot(t(1:end-1),ukf_err,LineWidth=2)
plot(t(1:end-1),mukf_err,LineWidth=2)
xlabel("Time [s]")
ylabel("||\omega|| Error")
legend(["EKF","UKF","MUKF"])


end

