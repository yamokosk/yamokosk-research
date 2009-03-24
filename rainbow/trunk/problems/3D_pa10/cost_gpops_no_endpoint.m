function [endpoint, integrand, DerivEndpoint, DerivLagrange] = cost_gpops_no_endpoint(sol)

global CONSTANTS;
bounds = CONSTANTS.bounds;

Q = eye(12);
R = eye(6);

tau0 = sol.initial.time;
y0 = sol.initial.state;
y = sol.state;
mu = sol.control;
tau = sol.time;

endpoint = 0;
integrand = dot(y,y*Q',2)+dot(mu,mu*R',2);

DerivEndpoint = [zeros(1,length(y0)), zeros(1,length(tau0)), ...
                 zeros(1,length(y0)), zeros(1,length(tau0))];
DerivLagrange = [2*y*Q', 2*mu*R', zeros(size(tau))];