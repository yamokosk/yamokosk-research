function [fitness] = neval(state, target, udata)

params = udata.params;

% Constants and parameters used to tune metric
D_ref = params.D_ref; %0.09;       % Approximate diameter of reference object in meters
S_width = params.S_width; %0.179;    % Width of sensor

%% Pull out information from inputs
% Create target T-matrix
p = target(1:3,1);
ori = target(4:6,1);
v = target(7:12,1);
T_wcs_t = rotk(ori);
T_wcs_t(1:3,4) = p;

% Create source robot T-matrix
[x_src, x_sen, feasible] = metricToJointSpaceMap(state, udata);
if ( ~feasible )
    fitness = 0;
end
T_src = kine(udata.rsrc,x_src(1:6,1));
T_sen = kine(udata.rsen,x_sen(1:6,1));

%% Compute intermediate variables and visibility parameters
P_src = T_src(1:3,4);     % Vector to source EE
P_sen = T_sen(1:3,4);     % Vector to sensor EE
Z_src = T_src(1:3,3);     % Actual image axis
P_src2t = p - P_src;      % Vector from source to target
P_sen2t = p - P_sen;      % Vector from sensor to target
n_hat = T_wcs_t(1:3,3);   % Desired image axis      

% Compute visibility parameters: r, d, theta, phi, A_hat, and psi
r = norm( P_src2t );                    % Distance between target/source
d = state(13,1);                        % Distance between source/sensor
phi = acos( dot(-P_src2t, n_hat)/r );    % Angle between desird view vector and actual image axis
ctheta = dot(Z_src, P_src2t)/r; % Cosine of angle between image axis and target center
theta = acos(ctheta);                   % Angle between image axis and target center
A_hat = (pi/4)*( D_ref * d/(S_width*r*ctheta) )^2;  % Normalized image area

%% Metrics
f_A = gompertz(1, -3.3058, -10.4147, A_hat);
f_phi = gaussian_rbf(phi,0,5);
f_theta = gaussian_rbf(theta,0,5);
% fitness = f_phi*f_A*f_theta;

%% Safety metric
J_sen = jacobian(udata.rsen, x_sen(1:6,1));
v_sen = J_sen * x_sen(7:12,1);

f_v_sen = 1;
check = dot(v_sen(1:3,1),P_sen2t);
if ( check > 0 )
    f_v_sen = (pi - 2*check)/pi;
end

J_src = jacobian(udata.rsrc,x_src(1:6,1));
v_src = J_src * x_src(7:12,1);

f_v_src = 1;
check = dot(v_src(1:3,1),P_src2t);
if ( check > 0 );
    f_v_src = (pi - 2*check)/pi;
end

fitness = f_phi*f_A*f_theta*f_v_sen*f_v_src;