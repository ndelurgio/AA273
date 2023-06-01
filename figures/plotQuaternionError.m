function plotQuaternionError(t,x,mu_ekf4quat,mu_ukf,mu_mukf)

q_ekf4quat = mu_ekf4quat(1:4,:);
q_ukf = mu_ukf(1:4,:);
q_mukf = mu_mukf(1:4,:);
q_true = x(1:4,1:end-1);

ekf4quat_err = vecnorm(q_ekf4quat-q_true,2,1);
ukf_err = vecnorm(q_ukf-q_true,2,1);
mukf_err = vecnorm(q_mukf-q_true,2,1);

figure
hold on;
plot(t(1:end-1),ekf4quat_err,LineWidth=2)
plot(t(1:end-1),ukf_err,LineWidth=2)
plot(t(1:end-1),mukf_err,LineWidth=2)
xlabel("Time [s]")
ylabel("Norm Error")
legend(["EKF 4 Quaternion","UKF","MUKF"])


end

