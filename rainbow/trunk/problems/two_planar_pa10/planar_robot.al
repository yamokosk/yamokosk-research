%------------------------------------------------------------
%	
%   Planar robot
%
%------------------------------------------------------------
AUTOZ ON

% -----------------------------------------------------------
%   Physical declarations: Newtonian, bodies, points, frames
newtonian	N					% Newtonian reference frame
bodies		AE,BE,CE			% 3 rigid links
points		No,J{3}				% 6 single axis joints

% -----------------------------------------------------------
%   Mathematical declarations: variables, constants, mass, inertia
variables 	q{3}', u{3}'		% Generalized coordinates
constants	LN,L1,L2			% Link lengths
constants	R					% Radius of ball on end of EE
mass		AE=m1,BE=m2,CE=m3
inertia		AE,0,0,m1*L1^2/12
inertia		BE,0,0,m2*L2^2/12
inertia		CE,0,0,m3*R^2/2.5
variables	Tf{3}				% Joint friction
constants	T{3}				% Applied joint torques
constants	C{3}, V{3}, Beta	% Friction constants

% -----------------------------------------------------------
%   Geometry relating unit vectors
% Source robot
simprot(N, AE, 3, q1)
simprot(AE,BE, 3, q2)
simprot(BE,CE, 3, q3)

% -----------------------------------------------------------
%   Position vectors
% Source robot
P_No_J1> = -LN*N2>
P_J1_J2> = L1*AE2>
P_J2_J3> = L2*BE2>

P_J1_AEo> = L1/2*AE2>
P_J2_BEo> = L2/2*BE2>
P_J3_CEo> = 0>

% -----------------------------------------------------------
%   Kinematical relationships
q1' = u1
q2' = u2
q3' = u3

% -----------------------------------------------------------
%   Angular velocities
angvel(N,AE)
angvel(AE,BE)
angvel(BE,CE)

% -----------------------------------------------------------
%   Angular accelerations
ALF_AE_N> = expand(dt(W_AE_N>,AE))
ALF_BE_N> = expand(dt(W_BE_N>,BE))
ALF_CE_N> = expand(dt(W_CE_N>,CE))

% -----------------------------------------------------------
%   Linear velocities
v_No_N> = 0>
v2pts(N,AE,No,AEo)
v2pts(N,AE,No,J1)
v2pts(N,BE,J1,BEo)
v2pts(N,BE,J1,J2)
v2pts(N,CE,J2,CEo)
v2pts(N,CE,J2,J3)

% -----------------------------------------------------------
%   Linear accelerations
a_No_N> = 0>
a2pts(N,AE,No,AEo)
a2pts(N,AE,No,J1)
a2pts(N,BE,J1,BEo)
a2pts(N,BE,J1,J2)
a2pts(N,CE,J2,CEo)
a2pts(N,CE,J2,J3)

% -----------------------------------------------------------
%   Internal forces (actuators, friction)
% Total torque for each joint (applied - friction)
Tf1 = C1 * tanh(Beta * u1) + V1 * u1
Tf2 = C2 * tanh(Beta * u2) + V2 * u2
Tf3 = C3 * tanh(Beta * u3) + V3 * u3

torque(N/AE,(T1 - Tf1)*AE3>)
torque(AE/BE,(T2 - Tf2)*BE3>)
torque(BE/CE,(T3 - Tf3)*CE3>)

% -----------------------------------------------------------
%   Equations of motion
zero = fr() + frstar()
kane()

% -----------------------------------------------------------
%   Inverse kinematics
%px[1] = dot(P_No_J1>,N1>)
%px[2] = dot(P_No_J2>,N1>)
%px[3] = dot(P_No_J3>,N1>)

%py[1] = dot(P_No_J1>,N2>)
%py[2] = dot(P_No_J2>,N2>)
%py[3] = dot(P_No_J3>,N2>)

%result = [px, py]

% -----------------------------------------------------------
%   Forward dynamics
result = [u1'; u2'; u3']

% -----------------------------------------------------------
%   Code generation
solve(zero,result)

dfdT = D(result,[T1,T2,T3])
dfdq = D(result,[q1,q2,q3])
dfdu = D(result,[u1,u2,u3])
output result, dfdT, dfdq, dfdu
code Algebraic() planar_robots_EOM.c