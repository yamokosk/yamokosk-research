function x = descale(y, x_lb, x_ub)
x = y .* (x_ub - x_lb) + x_lb;