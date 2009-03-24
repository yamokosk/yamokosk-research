function [endpoint, integrand, DerivEndpoint, DerivLagrange] = cost_gpops_no_endpoint_unscaled(sol)

Q = eye(12);
R = eye(6);

t0 = sol.initial.time;
x0 = sol.initial.state;
x = sol.state;
u = sol.control;
t = sol.time;

endpoint = 0;
integrand = dot(x,x*Q',2)+dot(u,u*R',2);

DerivEndpoint = [zeros(1,length(x0)), zeros(1,length(t0)), ...
                 zeros(1,length(x0)), zeros(1,length(t0))];
DerivLagrange = [2*x*Q', 2*u*R', zeros(size(t))];