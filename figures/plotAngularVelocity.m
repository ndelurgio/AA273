function plotAngularVelocity(t,w)
%PLOTQUATERNION Summary of this function goes here
%   Detailed explanation goes here
figure()
subplot 311
plot(t,rad2deg(w(1,:)))
ylabel("\omega_x [deg/s]")

subplot 312
plot(t,rad2deg(w(2,:)))
ylabel("\omega_y [deg/s]")

subplot 313
plot(t,rad2deg(w(3,:)))
ylabel("\omega_z [deg/s]")
xlabel("Time [s]")

end

