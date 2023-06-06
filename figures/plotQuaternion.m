function plotQuaternion(t,x,y,mu_ekf4quat,mu_ekf3quat,mu_ukf,mu_mukf)

t = t(1:end-1);
q = x(1:4,1:end-1);
figure()
subplot 411
hold on;
plot(t,q(1,:),Linewidth=2)
plot(t,y(1,:))
plot(t,mu_ekf4quat(1,:))
plot(t,mu_ekf3quat(1,:))
plot(t,mu_ukf(1,:))
plot(t,mu_mukf(1,:))
ylabel("q1")
legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 412
hold on;
plot(t,q(2,:),Linewidth=2)
plot(t,y(2,:))
plot(t,mu_ekf4quat(2,:))
plot(t,mu_ekf3quat(2,:))
plot(t,mu_ukf(2,:))
plot(t,mu_mukf(2,:))
ylabel("q2")
legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 413
hold on;
plot(t,q(3,:),Linewidth=2)
plot(t,y(3,:))
plot(t,mu_ekf4quat(3,:))
plot(t,mu_ekf3quat(3,:))
plot(t,mu_ukf(3,:))
plot(t,mu_mukf(3,:))
ylabel("q3")
legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])

subplot 414
hold on;
plot(t,q(4,:),Linewidth=2)
plot(t,y(4,:))
plot(t,mu_ekf4quat(4,:))
plot(t,mu_ekf3quat(4,:))
plot(t,mu_ukf(4,:))
plot(t,mu_mukf(4,:))
ylabel("q4")
legend(["True","Star Tracker","EKF 4 Quaternion","EKF 3 Quaternion","UKF","MUKF"])
xlabel("Time [s]")

% figure()
% plot(t,vecnorm(q,2,1))
% xlabel("Time [s]")
% ylabel("||q||")

end

