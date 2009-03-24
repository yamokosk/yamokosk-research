function [T, U, X, exitflag, exitmsg] = connect_gpops_no_endpoint(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, udata)

global CONSTANTS;

% Bounds on states and controls used later to scale the optimization problem.
bounds = struct('x_lb', x_lb, ...
                'x_ub', x_ub, ...
                'u_lb', u_lb, ...
                'u_ub', u_ub, ...
                't_lb', tspan(1), ...
                't_ub', tspan(2), ...
                'x2_ub', x_ub(7:12)./(1*(tspan(2)-tspan(1))), ...
                'x2_lb', x_lb(7:12)./(1*(tspan(2)-tspan(1))) );

CONSTANTS = struct('x0', x0, 'xf', xf, 'bounds', bounds);

% Generate a nice guess
t_guess = tspan';
x_guess = [x0'; xf'];
u_guess = zeros(2,6);

% Scale problem
[tauspan, y0, yf, y_lb, y_ub, mu_lb, mu_ub, tau_guess, y_guess, mu_guess] = ...
    scaleGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);

% Generate problem inputs
setup = setupGPOPS(tauspan, y0, yf, y_lb, y_ub, mu_lb, mu_ub, tau_guess, y_guess, mu_guess);

% Call GPOPS
output = gpops(setup);
sol = output.solution;

% Unscale so we get the proper answer
[T, X, U] = unScaleGPOPS(sol.time, sol.state, sol.control);
exitflag = output.SNOPT_info;
exitmsg = [];
% 
% h = figure();
% subplot(2,1,1)
% plot(T,X(:,1:6))
% subplot(2,1,2)
% plot(T,X(:,7:12))


function setup = setupGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess);
% Options
N = 14; % Number of nodes used to discretize and solve problem
direction = 'increasing';
autoscale = 'off';
derivativeMethod = 'analytic';
checkDerivatives = false;

% Problem name
name = 'PA10_state_connection';

% Functions structure
funcs = struct('cost', 'cost_gpops_no_endpoint', 'dae', 'dae_gpops');

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
