function [endpoint, integrand, DerivEndpoint, DerivLagrange] = cost_gpops_with_endpoint_unscaled(sol)

global CONSTANTS;

xf_d = CONSTANTS.xf;    % Desired terminus conditions

Q = eye(12);
R = eye(6);
S = diag([10;10;10;10;10;10;1;1;1;1;1;1]);

t0 = sol.initial.time;
xf = sol.terminal.state;    % Current scaled terminus conditions
e = xf - xf_d;

t = sol.time;
x = sol.state;
u = sol.control;

endpoint = dot(e,S*e);
integrand = dot(x,x*Q',2)+dot(u,u*R',2);

DerivEndpoint = [zeros(1,length(xf)), zeros(1,length(t0)), ...
                 e'*S, zeros(1,length(t0))];
DerivLagrange = [2*x*Q', 2*u*R', zeros(size(t))];