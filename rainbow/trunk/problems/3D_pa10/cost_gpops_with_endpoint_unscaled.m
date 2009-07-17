function [endpoint, integrand, DerivEndpoint, DerivLagrange] = cost_gpops_with_endpoint_unscaled(sol)

global CONSTANTS;

xf_d = CONSTANTS.xf;    % Desired terminus conditions
Nx = CONSTANTS.Nx;		% State scaling
Nu = CONSTANTS.Nu;		% Control effort scaling

Q = eye(12);			% State weight matrix
R = eye(6);				% Control effort weight matrix
S = diag([10;10;10;10;10;10;1;1;1;1;1;1]);		% Endpoint weight matrix	

Qn = Q*Nx;				% Normalized versions of the weight matrices above
Rn = R*Nu;
Sn = S*Nx;

t0 = sol.initial.time;
xf = sol.terminal.state;    % Current scaled terminus conditions
e = xf - xf_d;

t = sol.time;
x = sol.state;
u = sol.control;

endpoint = dot(e,Sn*e);
integrand = dot(x,x*Qn',2)+dot(u,u*Rn',2);

DerivEndpoint = [zeros(1,length(xf)), zeros(1,length(t0)), ...
                 e'*Sn, zeros(1,length(t0))];
DerivLagrange = [2*x*Qn', 2*u*Rn', zeros(size(t))];