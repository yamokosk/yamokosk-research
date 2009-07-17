function [endpoint, integrand] = cost_gpops_with_endpoint(sol)

global CONSTANTS;
bounds = CONSTANTS.bounds;
x0_d = CONSTANTS.x0;    % Desired initial conditions
xf_d = CONSTANTS.xf;    % Desired terminus conditions

Q = eye(12);
R = eye(6);
S = diag([10;10;10;10;10;10;1;1;1;1;1;1]);

tau0 = sol.initial.time;
x0 = sol.initial.state;
tauf = sol.terminal.time;
yf = sol.terminal.state;    % Current scaled terminus conditions
xf = descale(yf, bounds.x_lb, bounds.x_ub);
e = xf - xf_d;
err = scale(e, bounds.x_lb, bounds.x_ub); % Scale error

tau = sol.time;
y = sol.state;
mu = sol.control;

endpoint = dot(err,S*err);
integrand = dot(y,y*Q',2)+dot(mu,mu*R',2);

% DerivEndpoint = [zeros(1,length(x0)), zeros(1,length(t0)), ...
%                  xf'*S, zeros(1,length(tf), zeros(1,length(p))];
% DerivLagrange = [x*Q', u*R', ...
%                  zeros(length(t),length(p)), zeros(size(t))];