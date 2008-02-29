function xdot = planar_robots_ode(sol)
% get size of states
nrows = size(sol.states,1);
xdot = zeros(size(sol.states));

q_min = [-64, -107, -165] * (pi/180);
q_max = [124, 158, 165] * (pi/180);
qp_min = -1*[57, 114, 360] * (pi/180);
qp_max = [57, 114, 360] * (pi/180);
u_min = [4.64; 2.0; 0.29; 4.64; 2.0; 0.29]*-50;
u_max = [4.64; 2.0; 0.29; 4.64; 2.0; 0.29]*50;

x_min = [q_min'; qp_min'];
x_max = [q_max'; qp_max'];

qpp_min = -1*[25.1; 114.1; 2425.1];
qpp_max = [25.1; 114.1; 2425.1];

for n = 1:nrows
    xsrc_norm = sol.states(n,1:6)';
    xsen_norm = sol.states(n,7:12)';
    u_norm = sol.controls(n,:)';

    % Copy velocities
    xdot(n,1:3) = xsrc_norm(4:6);
    xdot(n,7:9) = xsen_norm(4:6);

    % States and control are normalized between -1 and 1.. need to expand
    % them back out.
    xsrc = ( (x_max - x_min) .* xsrc_norm + (x_max + x_min) ) ./ 2;
    xsen = ( (x_max - x_min) .* xsen_norm + (x_max + x_min) ) ./ 2;
    u = ( (u_max - u_min) .* u_norm + (u_max + u_min) ) ./ 2;
    
    qpp_src = planar_robot_eom(u(1:3), xsrc(1:3), xsrc(4:6));
    qpp_sen = planar_robot_eom(u(4:6), xsen(1:3), xsen(4:6));

    % Normalize the results
    xdot(n,4:6) = ((2*qpp_src - (qpp_max + qpp_min)) ./ (qpp_max - qpp_min))';
    xdot(n,10:12) = ((2*qpp_sen - (qpp_max + qpp_min)) ./ (qpp_max - qpp_min))';
end