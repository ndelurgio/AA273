function plotQuaternionError(t,x,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)

q_ekf4quat = mu_ekf4quat(1:4,:);
q_ekf3quat = mu_ekf3quat(1:4,:);
q_ukf = mu_ukf(1:4,:);
q_mekf = mu_mekf(1:4,:);
q_mukf = mu_mukf(1:4,:);
q_true = x(1:4,1:end-1);

ekf4quat_err = vecnorm(q_ekf4quat-q_true,2,1);
mekf_err = vecnorm(q_mekf-q_true,2,1);
% ekf3quat_err = vecnorm(q_ekf3quat-q_true,2,1);
ukf_err = vecnorm(q_ukf-q_true,2,1);
mukf_err = vecnorm(q_mukf-q_true,2,1);

% figure
% hold on;
% plot(t(1:end-1),ekf4quat_err,LineWidth=2)
% % plot(t(1:end-1),ekf3quat_err,LineWidth=2)
% plot(t(1:end-1),mekf_err,LineWidth=2)
% plot(t(1:end-1),ukf_err,LineWidth=2)
% plot(t(1:end-1),mukf_err,LineWidth=2)
% xlabel("Time [s]")
% ylabel("Norm Error")
% legend(["EKF","MEKF","UKF","MUKF"])

% T = len(mu_ekf4quat(1,:));
q_err_ekf = quatmultiply(quatconj(q_ekf4quat'),q_true');
ang_err_ekf = abs(2*acosd(q_err_ekf(:,4))-180);

q_err_ekf3 = quatmultiply(quatconj(q_ekf3quat'),q_true');
ang_err_ekf3 = abs(2*acosd(q_err_ekf3(:,4))-180);

q_err_mekf = quatmultiply(quatconj(q_mekf'),q_true');
ang_err_mekf = abs(2*acosd(q_err_mekf(:,4))-180);

q_err_ukf = quatmultiply(quatconj(q_ukf'),q_true');
ang_err_ukf = abs(2*acosd(q_err_ukf(:,4))-180);

q_err_mukf = quatmultiply(quatconj(q_mukf'),q_true');
ang_err_mukf = abs(2*acosd(q_err_mukf(:,4))-180);

figure
hold on;
grid on;
k = 10;
plot(t(1:end-1),movmean(ang_err_ekf,floor(length(ang_err_ekf3)/k)),LineWidth=2)
% plot(t(1:end-1),ekf3quat_err,LineWidth=2)
plot(t(1:end-1),movmean(ang_err_ekf3,floor(length(ang_err_ekf3)/k)),LineWidth=2)
plot(t(1:end-1),movmean(ang_err_mekf,floor(length(ang_err_ekf3)/k)),LineWidth=2)
plot(t(1:end-1),movmean(ang_err_ukf,floor(length(ang_err_ekf3)/k)),LineWidth=2)
plot(t(1:end-1),movmean(ang_err_mukf,floor(length(ang_err_ekf3)/k)),LineWidth=2)
xlabel("Time [s]")
ylabel("Angle Error [deg]")
legend(["EKF4","EKF3","MEKF","UKF","MUKF"])

% ang_err_mekf = zeros(T);
% ang_err_ukf = zeros(T);
% ang_err_mukf = zeros(T);
% for i = 1:T
%     ang_err_ekf(i) = 2*acosd(quatmultiply(quadconj(ang_err_ekf(:,i)'),x))
% end

disp("Mean EKF4 Error = " + string(mean(ang_err_ekf)))
disp("Mean EKF3 Error = " + string(mean(ang_err_ekf3)))
disp("Mean MEKF Error = " + string(mean(ang_err_mekf)))
disp("Mean UKF Error = " + string(mean(ang_err_ukf)))
disp("Mean MUKF Error = " + string(mean(ang_err_mukf)))


end

