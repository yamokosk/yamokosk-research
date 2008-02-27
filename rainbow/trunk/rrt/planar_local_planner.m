function [Ni,We] = planar_local_planner(Ne, Nr, Prob)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
t0 = Ne(1); x0 = Ne(2:end);
tf = Nr(1); xf = Nr(2:end);

[U,Path] = connect_min_effort(to, Xo, tf, Xf, @planar_robots_ode, @planar_cost, Prob.userdata);

branch.Path = Path;
branch.control = U;