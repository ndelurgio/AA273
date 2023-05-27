function plotAngularVelocity(t,w,y)

t = t(1:end-1);
w = w(:,1:end-1);

figure()
subplot 311
hold on;
plot(t,rad2deg(w(1,:)))
plot(t,rad2deg(y(5,:)))
ylabel("\omega_x [deg/s]")
legend(["True","Measured"])

subplot 312
hold on;
plot(t,rad2deg(w(2,:)))
plot(t,rad2deg(y(6,:)))
ylabel("\omega_y [deg/s]")
legend(["True","Measured"])

subplot 313
hold on;
plot(t,rad2deg(w(3,:)))
plot(t,rad2deg(y(7,:)))
ylabel("\omega_z [deg/s]")
xlabel("Time [s]")
legend(["True","Measured"])

end

