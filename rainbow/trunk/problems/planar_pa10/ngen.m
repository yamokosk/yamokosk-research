function xr = ngen(target, udata)

%% Constants used to tune metric
phi_max = pi/4;     % Angle between desired target normal and sensor location.
W_ref = 0.05;       % Width of reference object in meters
H_ref = 0.05;       % Height of reference object in meters
N = 1024;           % Number of pixels in row of sensor
Rmin = 0.20;        % Min percentage object occupies in image
Rmax = 1;        % Max percentage object occupies in image
theta_max = pi/6;   % FOV of sensor

% Generate random visibility parameters
phi = (2*rand(1)-1)*phi_max;
theta = (2*rand(1)-1)*(theta_max/2);
ct = cos(theta);
tt_max = tan(theta_max);

% Compute bounds on r
rmin = (N/(2*ct*tt_max))*sqrt(W_ref*H_ref/(Rmax*N^2));
rmax = (N/(2*ct*tt_max))*sqrt(W_ref*H_ref/(Rmin*N^2));
r = (rmax-rmin)*rand(1) + rmin;

% Rotate n_hat by phi which gives the direction of P_t2s
n_hat = [target(5:6); 0];
T_phi = rotz(phi); R_phi = T_phi(1:3,1:3); % Should this be a negative rotation?
n_t2s = R_phi * n_hat;
n_s2t = -n_t2s;
P_s2t = r*n_s2t;

% Now rotate P_s2t by theta to determine s_hat
T_theta = rotz(theta); R_theta = T_theta(1:3,1:3);
s_hat = R_theta*P_s2t;
s_hat = s_hat/norm(s_hat);

% And calculate actual position of sensor
ti = [target(1:2,1); 0];
P_s = ti - P_s2t;

% Now reconstruct full t-matrix for sensor EE
zaxis = [0;0;1];
y_hat = cross(zaxis,s_hat);
T_src_EE = [s_hat, y_hat, zaxis, P_s; 0, 0, 0, 1];

% Use inverse kinematics to compute src joint angles
%[Q, converged] = inv_kin(udata.rsrc, T_src_EE);
figure(1)
hold on
plot(P_s(1),P_s(2),'*')
plot(target(1),target(2),'o')
line([P_s(1); P_s(1) + s_hat(1)],[P_s(2); P_s(2) + s_hat(2)])
line([target(1); target(1) + n_hat(1)],[target(2); target(2) + n_hat(2)])
axis square

xr = rand(2,1);