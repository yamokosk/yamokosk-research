function xr = ngen(t, target, udata)
% State generator
%
%   x = [position_src_EE; orientation_src_EE; vel_src_EE;
%   distance_between_src_sen; time];
params = udata.params;

%% Constants and parameters used to tune metric
phi_std = params.phi_std; %(pi/4)/6; % Deviation about target's local x-axis
rmin = params.r_min;
rmax = params.r_max;
D_ref = params.D_ref; %0.09;       % Approximate diameter of reference object in meters
S_width = params.S_width; %0.179;    % Width of sensor
Rmin = params.Rmin; %0.30;        % Min percentage object occupies in image
Rmax = params.Rmax; %0.80;        % Max percentage object occupies in image

theta_std = params.theta_std; %0.0698/6; % Max FOV/2 ~ 4 degrees.

%% Pull out information from inputs
p = target(1:3,1);
ori = target(4:6,1);
v = target(7:12,1);

T_wcs_t = rotk(ori);
T_wcs_t(1:3,4) = p;

%% Generate random visibility parameters
r = (rmax-rmin)*rand(1)+rmin; %0.6;            % Distance between source and target
theta = theta_std*randn(1);
ct = cos(theta);

l_min = ((r*2*S_width*ct)/D_ref) * sqrt(Rmin/pi) - r;
l_min = max(.3, l_min);
l_max = ((r*2*S_width*ct)/D_ref) * sqrt(Rmax/pi) - r;
l = (l_max-l_min)*rand(1) + l_min;
d = r + l;

phi = phi_std*randn(1);

% From T_wcs_t to T_src
%   1. Create a deviation 'cone' by rotating [-pi,pi] about T_wcs_t local Z then by
%   phi about new local X
%   2. Then walk along new local Z by r. This is the src2tg vector.
%   3. Once at the src origin, again create a deviation 'cone' by rotating
%   [-pi,pi] about the local Z and then by theta about the new local Y.
%a = rand(2,1)*2*pi - pi;
%T_src = T_wcs_t * rotz(a(1)) * rotx(phi) * transl(0,0,r) * rotz(a(2)) * roty(theta);
T_src = T_wcs_t * rotx(phi) * transl(0,0,r) * roty(theta);
T_src = T_src * roty(pi); % Final rotation needed because Z-axis points out of end-effector.

k = irotk(T_src);
xr = [T_src(1:3,4); k; v; d; t];

% Plot to see if this is working correctly
%checkPlot(xr,target,udata);

% Check feasibility
[x_src, x_sen, feasible] = metricToJointSpaceMap(xr, udata);
if ( ~feasible )
    xr = [];
end


function checkPlot(xr, target, udata)

T_src = rotk(xr(4:6,1));
T_src(1:3,4) = xr(1:3,1);
P_src = T_src(1:3,4);

T_sen = T_src * transl(0,0,xr(13)) * roty(pi);
P_sen = T_sen(1:3,4);

figure(1)
hold on;
grid on;
len = 0.1;

% Source
plot3(udata.rsrc.T_f_base(1,4), udata.rsrc.T_f_base(2,4), udata.rsrc.T_f_base(3,4), 'ro');
text(udata.rsrc.T_f_base(1,4), udata.rsrc.T_f_base(2,4), udata.rsrc.T_f_base(3,4), 'SRC');
plot3(P_src(1),P_src(2),P_src(3), 'rx');
text(P_src(1), P_src(2),P_src(3), 'EE_{src}');
s_hat = T_src(1:3,3);
x = [P_src(1); P_src(1) + (len*s_hat(1))];
y = [P_src(2); P_src(2) + (len*s_hat(2))];
z = [P_src(3); P_src(3) + (len*s_hat(3))];
line(x,y,z,'Color','red');
text(P_src(1) + (len/2 * s_hat(1)), P_src(2) + (len/2 * s_hat(2)), P_src(3) + (len/2 * s_hat(3)), 'Z_{src}');

% Sensor sensor axis
plot3(udata.rsen.T_f_base(1,4), udata.rsen.T_f_base(2,4), udata.rsen.T_f_base(3,4), 'go');
text(udata.rsen.T_f_base(1,4), udata.rsen.T_f_base(2,4), udata.rsen.T_f_base(3,4), 'SEN');
plot3(P_sen(1), P_sen(2), P_sen(3), 'gx');
text(P_sen(1), P_sen(2), P_sen(3), 'EE_{sen}');
z_sen_hat = T_sen(1:3,3);
x = [P_sen(1); P_sen(1) + (len*z_sen_hat(1))];
y = [P_sen(2); P_sen(2) + (len*z_sen_hat(2))];
z = [P_sen(3); P_sen(3) + (len*z_sen_hat(3))];
line(x,y,z,'Color','green');
text(P_sen(1) + (len/2 * z_sen_hat(1)), P_sen(2) + (len/2 * z_sen_hat(2)), P_sen(3) + (len/2 * z_sen_hat(3)), 'Z_{sen}');


% Current target and its normal axis
ti = target(1:3,1);
T_target = rotk(target(4:6,1));
plot3(ti(1), ti(2), ti(3), 'ko')
%arrow('Start', ti, 'Stop', ti + (len * n_hat), 'Length', 5, 'Width', 1);
n_hat = T_target(1:3,3);
x = [ti(1); ti(1) + (len*n_hat(1))];
y = [ti(2); ti(2) + (len*n_hat(2))];
z = [ti(3); ti(3) + (len*n_hat(3))];
line(x,y,z,'Color','black');

axis([-1.2 1.4 -0.7 0.7 0, 1]);

pause;