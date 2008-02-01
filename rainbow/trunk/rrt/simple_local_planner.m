function branch = simple_local_planner(N0,Nf)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
t0 = N0(1); X0 = N0(2:end);
tf = Nf(1); Xf = Nf(2:end);
[U,Path] = connect(to, Xo, tf, Xf, N, @planar_robots_ode);

branch.Path = Path;
branch.control = U;