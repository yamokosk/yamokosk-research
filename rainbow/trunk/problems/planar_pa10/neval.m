function fitness = neval(state, target, udata)

% Constants and parameters used to tune metric
phi_max = pi/4;     % Angle between desired target normal and sensor location.
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
ti = [target(1:2,1); 0];
P_src = T_F_src(1:3,4);
P_sen = T_F_sen(1:3,4);

P_src2sen = P_sen - P_src;

P_t2src = P_src - ti;
R_F_src = T_F_src(1:3,1:3);

n_hat = [target(5:6,1); 0]; % Desired target normal
s_hat = R_F_src(1:3,3);     % Image axis

% Compute visibility parameters
r = norm( P_t2src );      % Distance between target/source
d = norm(P_src2sen);    % Distance between source/sensor
cphi = dot(P_t2src, n_hat)/r;   % Angle between target normal and vector between target and sourceEE
if ( isabout(abs(cphi), 1, 1e-6) )
    phi = 0;
else
    phi = acos( abs( cphi ) );
end
ct = dot(P_src2sen, -P_t2src)/(r*d);   % Cosine of angle between image axis and target center

% Compute metric
if phi < phi_max
    f_phi = gaussian_rbf(phi,0,5);
else
    f_phi = 0;
end

R_hat = (pi/4)*( (D_ref * d)/(S_width*r*ct) )^2;

% Image area metric from Kehoe's thesis example. Eq 4-43, pp. 112
%f_A = f_phi * ( 1 - exp( -50*((0.99*(R_hat-Rmin))/(Rmax-Rmin) + 0.01) ) );

% My image area metric
f_A = gompertz(1, -8, -8, R_hat);
%fitness = f_phi*f_A;
fitness = f_phi;


function test = isabout(val, tval, eps)
if ( abs(val - tval) < eps )
    test = true;
else
    test = false;
end