function plotQuaternion(t,q,y)

t = t(1:end-1);
q = q(:,1:end-1);
figure()
subplot 411
hold on;
plot(t,q(1,:))
plot(t,y(1,:))
ylabel("q1")
legend(["True","Measured"])

subplot 412
hold on;
plot(t,q(2,:))
plot(t,y(2,:))
ylabel("q2")
legend(["True","Measured"])

subplot 413
hold on;
plot(t,q(3,:))
plot(t,y(3,:))
ylabel("q3")
legend(["True","Measured"])

subplot 414
hold on;
plot(t,q(4,:))
plot(t,y(4,:))
ylabel("q4")
legend(["True","Measured"])
xlabel("Time [s]")

figure()
plot(t,vecnorm(q,2,1))
xlabel("Time [s]")
ylabel("||q||")

end

