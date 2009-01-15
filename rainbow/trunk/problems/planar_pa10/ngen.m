function xr = ngen(target, udata)

%% Constants and parameters used to tune metric
phi_max = pi/4;     % Angle between desired target normal and sensor location.
D_ref = 0.09;       % Approximate diameter of reference object in meters
S_width = 0.179;    % Width of sensor
Rmin = 0.30;        % Min percentage object occupies in image
Rmax = 0.80;        % Max percentage object occupies in image
r = 0.6;            % Distance between source and target
theta_max = 0.0698; % Max FOV/2 ~ 4 degrees.

%% Generate random visibility parameters
theta = (2*rand(1)-1)*theta_max;
ct = cos(theta);

d_min = ((r*2*S_width*ct)/D_ref) * sqrt(Rmin/pi);
d_min = max(r+0.1, d_min);
d_max = ((r*2*S_width*ct)/D_ref) * sqrt(Rmax/pi);
d = (d_max-d_min)*rand(1) + d_min;

phi = (2*rand(1)-1)*phi_max;
%phi_d = phi * (180/pi);

%% Rotate n_hat by phi which gives the direction of P_t2s
n_hat = [target(5:6); 0];
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

% Plot to see if this is working correctly
% figure(1)
% hold on
% len = 0.1;
% % Source
% plot(udata.rsrc.T_f_base(1,4), udata.rsrc.T_f_base(2,4), 'ro');
% text(udata.rsrc.T_f_base(1,4), udata.rsrc.T_f_base(2,4), 'SRC');
% plot(P_src(1), P_src(2), 'rx');
% text(P_src(1), P_src(2), 'EE_{src}');
% x = [P_src(1); P_src(1) + (len*s_hat(1))];
% y = [P_src(2); P_src(2) + (len*s_hat(2))];
% line(x,y,'Color','red');
% text(P_src(1) + (len/2 * s_hat(1)), P_src(2) + (len/2 * s_hat(2)), 'Z_{src}');
% 
% % Sensor sensor axis
% plot(udata.rsen.T_f_base(1,4), udata.rsen.T_f_base(2,4), 'go');
% text(udata.rsen.T_f_base(1,4), udata.rsen.T_f_base(2,4), 'SEN');
% plot(P_sen(1), P_sen(2), 'gx');
% text(P_sen(1), P_sen(2), 'EE_{sen}');
% x = [P_sen(1); P_sen(1) + (len*z_sen_hat(1))];
% y = [P_sen(2); P_sen(2) + (len*z_sen_hat(2))];
% line(x,y,'Color','green');
% text(P_sen(1) + (len/2 * z_sen_hat(1)), P_sen(2) + (len/2 * z_sen_hat(2)), 'Z_{sen}');
% 
% % Current target and its normal axis
% plot(target(1), target(2), 'k*')
% %arrow('Start', ti, 'Stop', ti + (len * n_hat), 'Length', 5, 'Width', 1);
% x = [ti(1); ti(1) + (len*n_hat(1))];
% y = [ti(2); ti(2) + (len*n_hat(2))];
% line(x,y,'Color','black');
% axis square

% Use inverse kinematics to compute src joint angles
[Qsrc, converged] = inv_kin(udata.rsrc, T_src_EE);
if (~converged)
    xr = [];
    return;
end

% Use inverse kinematics to compute src joint angles
[Qsen, converged] = inv_kin(udata.rsen, T_sen_EE);
if (~converged)
    xr = [];
    return;
end

% If inverse kinematics worked out, now compute the required joint
% velocities
vd = [target(3:4,1); 0; 0; 0; 0];
J_src = jacobian(udata.rsrc, Qsrc(:,1));
J_sen = jacobian(udata.rsen, Qsen(:,1));
% J_src_inv = jacobian_DLS_inv(udata.rsrc, Qsrc(:,1));
% J_sen_inv = jacobian_DLS_inv(udata.rsen, Qsen(:,1));
J_src_inv = pinv(J_src);
J_sen_inv = pinv(J_sen);


qp_src = J_src_inv * vd;
qp_sen = J_sen_inv * vd;

% Finally.. new rand state
xr = [Qsrc(:,1); qp_src; Qsen(:,1); qp_sen; target(end)];