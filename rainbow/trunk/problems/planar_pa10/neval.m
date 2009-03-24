function [fitness, f_A, f_phi, f_theta, f_psi] = neval(state, target, udata)

% Constants and parameters used to tune metric
D_ref = 0.09;       % Approximate diameter of reference object in meters
S_width = 0.179;    % Width of sensor
Rmin = 0.30;        % Min percentage object occupies in image
Rmax = 0.90;        % Max percentage object occupies in image

% Compute some stuff from the current state
q_src = state(1:3,1);
T_F_src = kine(udata.rsrc, q_src, 6);
q_sen = state(7:9,1);
T_F_sen = kine(udata.rsen, q_sen, 6);

% Compute intermediate variables and vectors
ti = [target(1:2,1); 0];    % Target position
P_src = T_F_src(1:3,4);     % Vector to source EE
P_sen = T_F_sen(1:3,4);     % Vector to sensor EE
P_src2sen = P_sen - P_src;  % Vector from source to sensor EE
P_src2t = ti - P_src;       % Vector from target to source
vt = [target(3); target(4); 0; 1];    % Target velocity
n_hat_h = rotz((pi/2) + target(5)) * vt;      % Desired image-axis vector
n_hat = [n_hat_h(1:2); 0]/norm(n_hat_h(1:2));
d_hat = T_F_sen(1:3,3);     % Detector normal

% Compute visibility parameters: r, d, theta, and phi
r = norm( P_src2t );    % Distance between target/source
d = norm( P_src2sen );  % Distance between source/sensor
psi = acos( dot(P_src2sen, -d_hat)/d );     % Angle between 
phi = acos( dot(P_src2t, n_hat)/r );            % Angle between desird view vector and actual image axis
ctheta = dot(P_src2sen, P_src2t)/(r*d);    % Cosine of angle between image axis and target center
theta = acos(ctheta);
A_hat = (pi/4)*( D_ref * d/(S_width*r*ctheta) )^2;    % Normalized image area

% Metrics
f_A = gompertz(1, -8, -8, A_hat);
f_phi = gaussian_rbf(phi,0,5);
f_theta = gaussian_rbf(theta,0,5);
f_psi = gaussian_rbf(psi,0,5);
fitness = f_phi*f_A*f_theta*f_psi;
