function [dfdu,dfdx] = planar_robots_ode_diff(x, u)
% Robot parameters
theta = [2; 1; 0.5; 0.25; 1.5; 1; 0.25; 0.2*ones(6,1); 0.1*ones(6,1); 10];

% Compute x_dot
[dfdu,dfdx] = planar_robots_diff(u, x(1:6,1), x(7:12,1), theta);