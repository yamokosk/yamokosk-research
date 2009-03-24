function [endpoint, integrand] = cost_gpops(sol)

global CONSTANTS;
bounds = CONSTANTS.bounds;

Q = eye(12);
R = eye(6);

y = sol.state;
mu = sol.control;

endpoint = 0;
integrand = dot(y,y*Q',2)+dot(mu,mu*R',2);

% DerivEndpoint = [zeros(1,length(x0)), zeros(1,length(t0)), ...
%                  xf'*S, zeros(1,length(tf), zeros(1,length(p))];
% DerivLagrange = [x*Q', u*R', ...
%                  zeros(length(t),length(p)), zeros(size(t))];