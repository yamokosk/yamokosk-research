function branch = simple_local_planner(No,Nf)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
[tf,U,X] = connect(0, No, Nf, @planar_robots_ode);
branch.Nf = X(:,end);
branch.control = U;
branch.weight = tf;