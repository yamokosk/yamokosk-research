function x_dot = planar_robots_ode(x, u)
% Robot parameters
theta = [2; 1; 0.5; 0.25; 1.5; 1; 0.25; 0.2*ones(6,1); 0.1*ones(6,1); 10];

% Compute x_dot
x_dot = zeros(size(x));
x_dot(1:6,1) = x(7:12,1);
x_dot(7:12,1) = planar_robots_eom(u, x(1:6,1), x(7:12,1), theta);