function y = scale(x, x_lb, x_ub)
y = (x - x_lb) ./ (x_ub - x_lb);