function [T, U, X, exitflag, exitmsg] = connect_gpops_with_endpoint_unscaled(t0, tf, x0, xf, x_lb, x_ub, u_lb, u_ub, robot_model, testing)

global CONSTANTS;

% Matrix use to normalize difference calculations during optimization
Nx = diag( (x_ub - x_lb).^2 );
Nx = inv(Nx);
Nu = diag( (u_ub - u_lb).^2 );
Nu = inv(Nu);

CONSTANTS = struct('x0', x0, 'xf', xf, 'Nx', Nx, 'Nu', Nu);

% Generate a nice guess
tspan = [t0, tf];
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

if (testing)
	close all;
	h = figure(1);
	for n = 1:6
		subplot(3,2,n);
		plot(T,U(:,n));
		line([t0; tf], [u_lb(n); u_lb(n)]);
		line([t0; tf], [u_ub(n); u_ub(n)]);
		title(['\tau_' num2str(n)]);
	end
	
	h = figure(2);
	for n = 1:6
		subplot(3,2,n);
		hold on;
		plot(tf,xf(n),'*');
		plot(T,X(:,n));
		title(['q_' num2str(n)]);
	end
end



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
