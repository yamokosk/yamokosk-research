function xdot = planar_robots_ode(sol)
% Robot parameters
theta = [2; 1; 0.5; 0.25; 1.5; 1; 0.25; 0.2*ones(6,1); 0.1*ones(6,1); 10];

% get size of states
nrows = size(sol.states,1);
xdot = zeros(size(sol.states));

for n = 1:nrows
    q = [sol.states(n,1:3)'; sol.states(n,7:9)];
    qp = [sol.states(n,4:6)'; sol.states(n,10:12)];
    u = sol.control(n,:)';
    xdot(n,1:3) = sol.states(n,4:6);
    xdot(n,7:9) = sol.states(n,10:12);
    qpp = planar_robots_eom(u, q, qp, theta);
    xdot(n,4:6) = qpp(1:3)';
    xdot(n,10:12) = qpp(4:6)';
end