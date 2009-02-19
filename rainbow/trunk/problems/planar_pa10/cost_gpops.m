function [endpoint, integrand] = cost_gpops(sol)

%global xf_desired;

Q = eye(12);
R = eye(6);
S = eye(12);

t0 = sol.initial.time;
x0 = sol.initial.state;
tf = sol.terminal.time;
xf = sol.terminal.state;
%e = xf - xf_desired;

t = sol.time;
x = sol.state;
u = sol.control;
p = sol.parameter;

endpoint = 0;%dot(e',S*e');
integrand = dot(x,x*Q',2)+dot(u,u*R',2);

% DerivEndpoint = [zeros(1,length(x0)), zeros(1,length(t0)), ...
%                  xf'*S, zeros(1,length(tf), zeros(1,length(p))];
% DerivLagrange = [x*Q', u*R', ...
%                  zeros(length(t),length(p)), zeros(size(t))];