function plotQuaternion(t,q)
%PLOTQUATERNION Summary of this function goes here
%   Detailed explanation goes here
figure()
subplot 411
plot(t,q(1,:))
ylabel("q1")

subplot 412
plot(t,q(2,:))
ylabel("q2")

subplot 413
plot(t,q(3,:))
ylabel("q3")

subplot 414
plot(t,q(4,:))
ylabel("q4")
xlabel("Time [s]")

figure()
plot(t,vecnorm(q,2,1))
xlabel("Time [s]")
ylabel("||q||")

end

