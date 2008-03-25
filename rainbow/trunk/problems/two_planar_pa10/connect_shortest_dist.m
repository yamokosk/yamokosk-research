function [T, U, X, exitflag, exitmsg] = connect_shortest_dist(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, N, odefun, costfun)

N = 2*N;
alpha = linspace(0,1,N);
T = linspace(tspan(1),tspan(2),N)';
X = zeros(length(x0), N);

for n = 1:N
    X(:,n) = (x0 + alpha(n) * (xf - x0));
end

U = zeros(size(X));

exitflag = 0;
exitmsg = 'finished successfully';