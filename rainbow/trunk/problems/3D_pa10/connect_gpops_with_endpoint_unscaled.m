function [T, U, X, exitflag, exitmsg] = connect_gpops_with_endpoint_unscaled(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, udata)

global CONSTANTS;

CONSTANTS = struct('x0', x0, 'xf', xf);

% Generate a nice guess
t_guess = tspan';
x_guess = [x0'; xf'];
u_guess = zeros(2,6);

% Generate problem inputs
setup = setupGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);

% Call GPOPS
output = gpops(setup);
sol = output.solution;

T = sol.time;
X = sol.state;
U = sol.control;
exitflag = output.SNOPT_info;
exitmsg = [];    


function setup = setupGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);
% Options
N = 15; % Number of nodes used to discretize and solve problem
direction = 'increasing';
autoscale = 'on';
derivativeMethod = 'analytic';
checkDerivatives = false;

% Problem name
name = 'PA10_state_connection';

% Functions structure
funcs = struct('cost', 'cost_gpops_with_endpoint_unscaled', 'dae', 'dae_gpops_unscaled');

% Limits structure
t0 = tspan(1); tf = tspan(2);
tmin = [t0, tf];    % tmin = [min(t) at start of phase, min(t) at terminus of phase]
tmax = [t0, tf];    % tmax = [max(t) at start of phase, max(t) at terminus of phase]
t_limit  = struct('min', tmin, 'max', tmax);
xmin = [x0, x_lb, x_lb];  % xmin = [min(x) at start of phase, min(x) during phase, min(x) at terminus of phase]
xmax = [x0, x_ub, x_ub];  % xmax = [max(x) at start of phase, max(x) during phase, max(x) at terminus of phase]
x_limit = struct('min', xmin, 'max', xmax);
u_limit = struct('min', u_lb, 'max', u_ub);
limits = struct('nodes', N, 'time', t_limit, 'state', x_limit, 'control', u_limit, 'parameter', struct('min',[],'max',[]));

% Guess structure
guess = struct('time', t_guess, 'state', x_guess, 'control', u_guess);

% Setup structure
setup = struct('name', name, 'funcs', funcs, 'limits', limits, 'guess', guess, ...
               'direction', direction, 'autoscale', autoscale, ...
               'derivatives', derivativeMethod, 'checkDerivatives', checkDerivatives);
