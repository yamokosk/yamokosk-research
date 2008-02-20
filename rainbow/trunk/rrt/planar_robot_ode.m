function x_dot = planar_robot_ode(x, u)
% Robot parameters
theta = [2; 1; 0.5; 0.25; 1.5; 1; 0.25; 0.2*ones(3,1); 0.1*ones(3,1); 10];

% Compute x_dot
x_dot = zeros(size(x));
x_dot(1:3,1) = x(4:6,1);
x_dot(4:6,1) = planar_robot_eom(u, x(1:3,1), x(4:6,1), theta);