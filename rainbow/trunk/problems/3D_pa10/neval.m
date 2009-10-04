function [fitness, f_A, f_phi, f_theta, f_psi] = neval(state, target, udata)

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
src_EE = state(1:12,1);
T_src = rotk(src_EE(4:6,1));
T_src(1:3,4) = src_EE(1:3,1);

%% Compute intermediate variables and visibility parameters
P_src = T_src(1:3,4);     % Vector to source EE
N_src2sen = T_src(1:3,3); % Actual image axis
P_src2t = p - P_src;      % Vector from target to source
n_hat = T_wcs_t(1:3,3);   % Desired image axis      
%d_hat = T_sen(1:3,3);     % Detector normal

% Compute visibility parameters: r, d, theta, phi, A_hat, and psi
r = norm( P_src2t );                    % Distance between target/source
d = state(13,1);                        % Distance between source/sensor
%psi = acos( dot(N_src2sen, -d_hat)/d ); % Angle between actual image axis and detector normal
phi = acos( dot(-P_src2t, n_hat)/r );    % Angle between desird view vector and actual image axis
ctheta = dot(N_src2sen, P_src2t)/r; % Cosine of angle between image axis and target center
theta = acos(ctheta);                   % Angle between image axis and target center
A_hat = (pi/4)*( D_ref * d/(S_width*r*ctheta) )^2;  % Normalized image area

%% Metrics
f_A = gompertz(1, -8, -8, A_hat);
f_phi = gaussian_rbf(phi,0,5);
f_theta = gaussian_rbf(theta,0,5);
f_psi = 1; %gaussian_rbf(psi,0,5);
fitness = f_phi*f_A*f_theta*f_psi;
%fitness = f_phi;
