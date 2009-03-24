% Unscale problem
function [t, X, U] = unScaleGPOPS(tau, Y, MU)

global CONSTANTS;

bounds = CONSTANTS.bounds;

N = length(tau);

t = descale(tau, bounds.t_lb, bounds.t_ub);
X = zeros(size(Y));
U = zeros(size(MU));
for n = 1:N
    X(n,:) = descale(Y(n,:), bounds.x_lb', bounds.x_ub');
    U(n,:) = descale(MU(n,:), bounds.u_lb', bounds.u_ub');
end