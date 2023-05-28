function plotGyroBias(t,b,mu_b)
t = t(1:end-1);
b = b(:,1:end-1);
figure()
subplot 311
hold on;
plot(t,rad2deg(b(1,:)))
plot(t,rad2deg(mu_b(1,:)))
legend(["True","Estimate"])
ylabel("b_x [deg/s]")

subplot 312
hold on;
plot(t,rad2deg(b(2,:)))
plot(t,rad2deg(mu_b(2,:)))
legend(["True","Estimate"])
ylabel("b_y [deg/s]")

subplot 313
hold on;
plot(t,rad2deg(b(3,:)))
plot(t,rad2deg(mu_b(3,:)))
legend(["True","Estimate"])
ylabel("b_z [deg/s]")
xlabel("Time [s]")
end

