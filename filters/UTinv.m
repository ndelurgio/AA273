function [mu,cov] = UTinv(x,w)
n = length(x(:,1));
mu = 0;
for i = 1:(2*n+1)
    mu = mu + w(1,i)*x(:,i);
end
cov = zeros(n,n);
for i = 1:(2*n+1)
    cov = cov + w(1,i)*(x(:,i)-mu)*(x(:,i)-mu)';
end


end

