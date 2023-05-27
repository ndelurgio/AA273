function plotGyroBias(t,b)
%PLOTGYROBIAS Summary of this function goes here
%   Detailed explanation goes here
figure()
subplot 311
plot(t,rad2deg(b(1,:)))
ylabel("b_x [deg/s]")

subplot 312
plot(t,rad2deg(b(2,:)))
ylabel("b_y [deg/s]")

subplot 313
plot(t,rad2deg(b(3,:)))
ylabel("b_z [deg/s]")
xlabel("Time [s]")
end

