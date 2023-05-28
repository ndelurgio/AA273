function plotAngularVelocity(t,w,y,mu_w)

t = t(1:end-1);
w = w(:,1:end-1);

figure()
subplot 311
hold on;
plot(t,rad2deg(w(1,:)))
plot(t,rad2deg(y(5,:)))
plot(t,rad2deg(mu_w(1,:)))
ylabel("\omega_x [deg/s]")
legend(["True","Measured","Estimate"])

subplot 312
hold on;
plot(t,rad2deg(w(2,:)))
plot(t,rad2deg(y(6,:)))
plot(t,rad2deg(mu_w(2,:)))
ylabel("\omega_y [deg/s]")
legend(["True","Measured","Estimate"])

subplot 313
hold on;
plot(t,rad2deg(w(3,:)))
plot(t,rad2deg(y(7,:)))
plot(t,rad2deg(mu_w(3,:)))
ylabel("\omega_z [deg/s]")
xlabel("Time [s]")
legend(["True","Measured","Estimate"])

end

