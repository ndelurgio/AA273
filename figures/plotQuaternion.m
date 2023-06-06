function plotQuaternion(t,x,y,mu_ekf4quat,mu_ekf3quat,mu_mekf,mu_ukf,mu_mukf)

t = t(1:end-1);
q = x(1:4,1:end-1);
figure()
subplot 411
hold on;
grid on;
plot(t,q(1,:),Linewidth=2)
plot(t,y(1,:))
plot(t,mu_ekf4quat(1,:))
plot(t,mu_ekf3quat(1,:))
plot(t,mu_mekf(1,:))
plot(t,mu_ukf(1,:))
plot(t,mu_mukf(1,:))
ylabel("q_1")
legend(["True","Star Tracker","EKF4","EKF3","MEKF","UKF","MUKF"])

subplot 412
hold on;
grid on;
plot(t,q(2,:),Linewidth=2)
plot(t,y(2,:))
plot(t,mu_ekf4quat(2,:))
plot(t,mu_ekf3quat(2,:))
plot(t,mu_mekf(2,:))
plot(t,mu_ukf(2,:))
plot(t,mu_mukf(2,:))
ylabel("q_2")
% legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","MEKF","UKF","MUKF"])

subplot 413
hold on;
grid on;
plot(t,q(3,:),Linewidth=2)
plot(t,y(3,:))
plot(t,mu_ekf4quat(3,:))
plot(t,mu_ekf3quat(3,:))
plot(t,mu_mekf(3,:))
plot(t,mu_ukf(3,:))
plot(t,mu_mukf(3,:))
ylabel("q_3")
% legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","MEKF","UKF","MUKF"])

subplot 414
hold on;
grid on;
plot(t,q(4,:),Linewidth=2)
plot(t,y(4,:))
plot(t,mu_ekf4quat(4,:))
plot(t,mu_ekf3quat(4,:))
plot(t,mu_mekf(4,:))
plot(t,mu_ukf(4,:))
plot(t,mu_mukf(4,:))
ylabel("q_4")
% legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","MEKF","UKF","MUKF"])
xlabel("Time [s]")

% figure()
% plot(t,vecnorm(q,2,1))
% xlabel("Time [s]")
% ylabel("||q||")

end

