% Scale problem
function [tauspan, y0, yf, y_lb, y_ub, mu_lb, mu_ub, tau_guess, y_guess, mu_guess] = ...
    scaleGPOPS(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, t_guess, x_guess, u_guess)

global CONSTANTS;
bounds = CONSTANTS.bounds;

% Scale states based on x0 and xf
y0 = scale(x0, bounds.x_lb, bounds.x_ub);
yf = scale(xf, bounds.x_lb, bounds.x_ub);
y_lb = scale(x_lb, bounds.x_lb, bounds.x_ub);
y_ub = scale(x_ub, bounds.x_lb, bounds.x_ub);

% Scale control based on u_lb and u_ub
mu_lb = scale(u_lb, bounds.u_lb, bounds.u_ub);
mu_ub = scale(u_ub, bounds.u_lb, bounds.u_ub);

% Scale time based on t0 and tf
tau0 = scale(tspan(1), bounds.t_lb, bounds.t_ub);
tauf = scale(tspan(2), bounds.t_lb, bounds.t_ub);
tauspan = [tau0, tauf];
tau_guess = scale(t_guess, bounds.t_lb, bounds.t_ub);

% Scale guesses
y_guess = zeros(size(x_guess));
mu_guess = zeros(size(u_guess));
for n = 1:size(y_guess,1)
    y_guess(n,:) = scale(x_guess(n,:), bounds.x_lb', bounds.x_ub');
    mu_guess(n,:) = scale(u_guess(n,:), bounds.u_lb', bounds.u_ub');
end