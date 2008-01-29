function robot_obj = load2DRobot(name)
% USAGE: robot_obj = load2DRobot('pa10');

% 2D robot constants
l1 = 1;
l2 = 1;
l3 = 1;

m1 = 1;
m2 = 1;
m3 = 1;

Izz1 = m1 * (l1/2)^2;
Izz2 = m2 * (l2/2)^2;
Izz3 = m3 * (l3/2)^2;

% Standard DH notation
alpha = zeros(1,3); % link twist angle
A = [l1, l2, l3];                 % link length
rx = [l1/2, l2/2, l3/2];
ry = zeros(1,3);
rz = zeros(1,3);

% Modified DH notation
% alpha = [ 0 -pi/2 0 pi/2 -pi/2 pi/2 ];
% A = [ 0 0 .450 0 0 0 ];
% rx = [0,0.129,0,0,0,0];                 % link COG
% ry = [0,-0.03,-0.0485,0,0.042,0];
% rz = [-0.1785,0,0,-0.1122,0,-0.048];

% Other link parameters, invariant to notation
theta = zeros(1,3);                    % link rotation angle
D = zeros(1,3);       % link offset distance
sigma = zeros(1,3);                    % joint type (0 for revolute)
offset = zeros(1,3);       % offset for theta (for zero-pose)

mass = [m1, m2, m3];   % mass of the link
Ixx = zeros(1,3);     % elements of link inertia tensor about the link COG
Iyy = zeros(1,3);
Izz = [Izz1, Izz2, Izz3];
Ixy = zeros(1,3);
Iyz = zeros(1,3);
Ixz = zeros(1,3);
Jm = zeros(1,3);        % armature inertia
G = 50 * ones(1,3);     % reduction gear ratio. joint speed/link speed
B = zeros(1,3);         % viscous friction, motor refered
Tc_plus = zeros(1,3);   % coulomb friction (positive rotation), motor refered
Tc_minus = zeros(1,3);  % coulomb friction (negative rotation), motor refered

min = [-pi, -pi, -pi];
max = [pi, pi, pi];

dyn_matrix = [alpha; A; theta; D; sigma; mass; rx; ry; rz; Ixx; Iyy; Izz; ...
              Ixy; Iyz; Ixz; Jm; G; B; Tc_plus; Tc_minus];
          
for i=1:length(A)
    L{i} = link(dyn_matrix(:,i)', 'standard');
    L{i}.offset = offset(i);
    L{i}.qlim = [min(i) max(i)];
end

robot_obj = robot(L,['PA10_' name]);