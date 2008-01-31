%------------------------------------------------------------
%	
%   Constrained planar robots
%
%------------------------------------------------------------
AUTOZ ON

% -----------------------------------------------------------
%   Physical declarations: Newtonian, bodies, points, frames
newtonian	N					% Newtonian reference frame
bodies		AE,BE,CE,AR,BR,CR	% 3 rigid links x2
points		No,J{6}				% 6 single axis joints

% -----------------------------------------------------------
%   Mathematical declarations: variables, constants, mass, inertia
variables 	q{6}', u{6}'		% Generalized coordinates
constants	LN,L1,L2			% Link lengths
constants	R					% Radius of ball on end of EE
mass		AE=m1,BE=m2,CE=m3
mass		AR=m1,BR=m2,CR=m3
inertia		AR,0,0,m1*L1^2/12
inertia		BR,0,0,m2*L2^2/12
inertia		CR,0,0,m3*R^2/2.5
inertia		AE,0,0,m1*L1^2/12
inertia		BE,0,0,m2*L2^2/12
inertia		CE,0,0,m3*R^2/2.5
variables	Tf{6}				% Joint friction
constants	T{6}				% Applied joint torques
constants	C{6}, V{6}, Beta	% Friction constants

% -----------------------------------------------------------
%   Geometry relating unit vectors
% Source robot
simprot(N, AE, 3, q1)
simprot(AE,BE, 3, q2)
simprot(BE,CE, 3, q3)

% Sensor robot
simprot(N, AR, 3, q4+pi)
simprot(AR,BR, 3, q5)
simprot(BR,CR, 3, q6)

% -----------------------------------------------------------
%   Position vectors
% Source robot
P_No_J1> = -LN*N2>
P_J1_J2> = L1*AE2>
P_J2_J3> = L2*BE2>

P_J1_AEo> = L1/2*AE2>
P_J2_BEo> = L2/2*BE2>
P_J3_CEo> = 0>

% Sensor robot
P_No_J4> = LN*N2>
P_J4_J5> = L1*AR2>
P_J5_J6> = L2*BR2>

P_J4_ARo> = L1/2*AR2>
P_J5_BRo> = L2/2*BR2>
P_J6_CRo> = 0>

% -----------------------------------------------------------
%   Kinematical relationships
q1' = u1
q2' = u2
q3' = u3
q4' = u4
q5' = u5
q6' = u6

% -----------------------------------------------------------
%   Angular velocities
angvel(N,AE)
angvel(AE,BE)
angvel(BE,CE)

angvel(N,AR)
angvel(AR,BR)
angvel(BR,CR)

% -----------------------------------------------------------
%   Angular accelerations
ALF_AE_N> = expand(dt(W_AE_N>,AE))
ALF_BE_N> = expand(dt(W_BE_N>,BE))
ALF_CE_N> = expand(dt(W_CE_N>,CE))

ALF_AR_N> = expand(dt(W_AR_N>,AR))
ALF_BR_N> = expand(dt(W_BR_N>,BR))
ALF_CR_N> = expand(dt(W_CR_N>,CR))

% -----------------------------------------------------------
%   Linear velocities
v_No_N> = 0>
v2pts(N,AE,No,AEo)
v2pts(N,AE,No,J1)
v2pts(N,BE,J1,BEo)
v2pts(N,BE,J1,J2)
v2pts(N,CE,J2,CEo)
v2pts(N,CE,J2,J3)

v2pts(N,AR,No,ARo)
v2pts(N,AR,No,J4)
v2pts(N,BR,J4,BRo)
v2pts(N,BR,J4,J5)
v2pts(N,CR,J5,CRo)
v2pts(N,CR,J5,J6)

% -----------------------------------------------------------
%   Linear accelerations
a_No_N> = 0>
a2pts(N,AE,No,AEo)
a2pts(N,AE,No,J1)
a2pts(N,BE,J1,BEo)
a2pts(N,BE,J1,J2)
a2pts(N,CE,J2,CEo)
a2pts(N,CE,J2,J3)

a2pts(N,AR,No,ARo)
a2pts(N,AR,No,J4)
a2pts(N,BR,J4,BRo)
a2pts(N,BR,J4,J5)
a2pts(N,CR,J4,CRo)
a2pts(N,CR,J4,J6)

% -----------------------------------------------------------
%   Internal forces (actuators, friction)
% Total torque for each joint (applied - friction)
Tf1 = C1 * tanh(Beta * u1) + V1 * u1
Tf2 = C2 * tanh(Beta * u2) + V2 * u2
Tf3 = C3 * tanh(Beta * u3) + V3 * u3
Tf4 = C4 * tanh(Beta * u4) + V4 * u4
Tf5 = C5 * tanh(Beta * u5) + V5 * u5
Tf6 = C6 * tanh(Beta * u6) + V6 * u6

torque(N/AE,(T1 - Tf1)*AE3>)
torque(AE/BE,(T2 - Tf2)*BE3>)
torque(BE/CE,(T3 - Tf3)*CE3>)

torque(N/AR,(T4 - Tf4)*AR3>)
torque(AR/BR,(T5 - Tf5)*BR3>)
torque(BR/CR,(T6 - Tf6)*CR3>)

% -----------------------------------------------------------
%   Equations of motion
zero = fr() + frstar()
kane()

% -----------------------------------------------------------
%   Forward dynamics
accelerations = [u1'; u2'; u3'; u4'; u5'; u6']

solve(zero,accelerations)
output accelerations
code Algebraic() planar_robots_eom.c

% -----------------------------------------------------------
%   Partials
%accelerations = [u1'; u2'; u3'; u4'; u5'; u6']
%solve(zero,accelerations)

%state = [q1,q2,q3,q4,q5,q6,u1,u2,u3,u4,u5,u6]
%f = [u1;u2;u3;u4;u5;u6;accelerations]
%control = [T1, T2, T3, T4, T5, T6]

%dfdu = D(f,control)
%dfdX = D(f,state)

%output dfdu, dfdX
%code Algebraic() planar_robots_diff.c