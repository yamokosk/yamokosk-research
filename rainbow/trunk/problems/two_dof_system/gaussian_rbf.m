function y = gaussian_rbf(x,c,beta)
% Gaussian radial basis function
r = norm(x-c);
y = exp(-beta*r^2);