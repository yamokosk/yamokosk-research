function fitness = neval(state, target, udata)

% Constants used to tune metric
phi_max = pi/4;     % Angle between desired target normal and sensor location.
W_ref = 0.05;       % Width of reference object in meters
H_ref = 0.05;       % Height of reference object in meters
N = 1024;           % Number of pixels in row of sensor
Rmin = 0.40;        % Min percentage object occupies in image
Rmax = 0.80;        % Max percentage object occupies in image
theta_max = pi/6;   % FOV of sensor

% Compute some stuff from the current state
q_src = state(1:3,1);
T_E_S = kine(udata.src, q_src);

% Compute intermediate variables and vectors
ti = [target(1:2,1); 0];
P_s = T_E_S(1:3,4);
P_s2t = ti - P_s;

R_E_S = T_E_S(1:3,1:3);

n_hat = [target(5:6,1); 0]; % Desired target normal
s_hat = R_E_S(1:3,1);

% Compute visibility parameters
r = norm( P_s2t );
phi = acos( dot(-P_s2t, n_hat)/r );
ct = dot(R_E_S*P_s2t, s_hat)/r;
theta = acos(ct);

% Compute metric
if phi < phi_max
    f_phi = 1;
else
    f_phi = 0;
end

tt_max = tan(theta_max);
W_hat = (W_ref/(r*ct))*(N/(2*tt_max));
H_hat = (H_ref/(r*ct))*(N/(2*tt_max));
A_hat = W_hat * H_hat;
R_hat = A_hat / (N*N);

% Image area metric from Kehoe's thesis example. Eq 4-43, pp. 112
f_A = f_phi * ( 1 - exp( -50*((0.99*(R_hat-Rmin))/(Rmax-Rmin) + 0.01) ) );

fitness = f_phi*f_A;