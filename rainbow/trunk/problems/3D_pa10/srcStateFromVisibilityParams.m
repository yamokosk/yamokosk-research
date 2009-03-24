function [x_src, x_sen, converged] = mapMeasureStateToRobotState(state)

src_EE, d, theta

n_hat_h = rotz((pi/2) + target(5)) * vt;      % Desired image-axis vector
n_hat = [n_hat_h(1:2); 0]/norm(n_hat_h(1:2));
T_phi = rotz(phi); R_phi = T_phi(1:3,1:3); % Should this be a negative rotation?
n_s2t = R_phi * n_hat;
P_src2t = r*n_s2t;

% Now rotate P_s2t by theta to determine s_hat
T_theta = rotz(theta); R_theta = T_theta(1:3,1:3);
s_hat = R_theta*P_src2t;
s_hat = s_hat/norm(s_hat);

% And calculate actual position of sensor
ti = [target(1:2,1); 0];
P_src = ti - P_src2t;

% Now reconstruct full t-matrix for sensor EE
yaxis = [0;0;1];
xaxis = cross(yaxis,s_hat);
T_src_EE = [xaxis, yaxis, s_hat, P_src; 0, 0, 0, 1];

% Now get the sensor's position. Construct it's full t-matrix
z_sen_hat = -s_hat; 
xaxis = cross(yaxis,z_sen_hat);
P_sen = P_src + d*s_hat;
T_sen_EE = [xaxis, yaxis, z_sen_hat, P_sen; 0, 0, 0, 1];



function [q, qp, converged] = computeRobotState(model, T_world_EE, v_target)

converged = false;

% Use inverse kinematics to compute joint angles
[q, converged] = inv_kin(model, T_world_EE);
if (~converged)
    q = []; qp = [];
    return;
end

J = jacobian(udata.rsrc, Qsrc(:,1));

J_src_inv = pinv(J_src);
qp_src = J_src_inv * vd;