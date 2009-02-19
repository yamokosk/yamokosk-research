function [T, U, X, exitflag, exitmsg] = connect_gpops(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, udata)

t_guess = tspan';
x_guess = [x0'; xf'];
u_guess = zeros(2,6);

% Generate problem inputs
setup = setupGPOPS( tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);

% Call GPOPS
sol = gpops(setup);

U = sol.control;
T = sol.time;
X = sol.state;
exitflag = [];
exitmsg = [];


function setup = setupGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);
% Options
N = 12; % Number of nodes used to discretize and solve problem
direction = 'increasing';
autoscale = 'on';
derivativeMethod = 'numerical';
checkDerivatives = false;

% Problem name
name = 'PA10_state_connection';

% Functions structure
funcs = struct('cost', 'cost_gpops', 'dae', 'dae_gpops');

% Limits structure
t0 = tspan(1); tf = tspan(2);
tmin = [t0, tf];    % tmin = [min(t) at start of phase, min(t) at terminus of phase]
tmax = [t0, tf];    % tmax = [max(t) at start of phase, max(t) at terminus of phase]
t_limit  = struct('min', tmin, 'max', tmax);
xmin = [x0, x_lb, xf];  % xmin = [min(x) at start of phase, min(x) during phase, min(x) at terminus of phase]
xmax = [x0, x_ub, xf];  % xmax = [max(x) at start of phase, max(x) during phase, max(x) at terminus of phase]
x_limit = struct('min', xmin, 'max', xmax);
u_limit = struct('min', u_lb, 'max', u_ub);
limits = struct('nodes', N, 'time', t_limit, 'state', x_limit, 'control', u_limit, 'parameter', struct('min',[],'max',[]));

% Guess structure
guess = struct('time', t_guess, 'state', x_guess, 'control', u_guess);

% Setup structure
setup = struct('name', name, 'funcs', funcs, 'limits', limits, 'guess', guess, ...
               'direction', direction, 'autoscale', autoscale, ...
               'derivatives', derivativeMethod, 'checkDerivatives', checkDerivatives);