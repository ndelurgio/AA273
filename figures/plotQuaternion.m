function plotQuaternion(t,q,y,mu_q)

t = t(1:end-1);
q = q(:,1:end-1);
figure()
subplot 411
hold on;
plot(t,q(1,:))
plot(t,y(1,:))
plot(t,mu_q(1,:))
ylabel("q1")
legend(["True","Measured","Estimate"])

subplot 412
hold on;
plot(t,q(2,:))
plot(t,y(2,:))
plot(t,mu_q(2,:))
ylabel("q2")
legend(["True","Measured","Estimate"])

subplot 413
hold on;
plot(t,q(3,:))
plot(t,y(3,:))
plot(t,mu_q(3,:))
ylabel("q3")
legend(["True","Measured","Estimate"])

subplot 414
hold on;
plot(t,q(4,:))
plot(t,y(4,:))
plot(t,mu_q(4,:))
ylabel("q4")
legend(["True","Measured","Estimate"])
xlabel("Time [s]")

figure()
plot(t,vecnorm(q,2,1))
xlabel("Time [s]")
ylabel("||q||")

end

