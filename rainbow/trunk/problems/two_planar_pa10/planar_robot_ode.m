function xdot = planar_robot_ode(sol)
xdot = zeros(size(sol.states));
xdot(:,1:3) = sol.states(:,4:6);
xdot(:,4:6) = planar_robot_eom( sol.controls', sol.states(:,1:3)', sol.states(:,4:6)' )';