function cost = stateCostFunction(state,T_lab_d, T_lab_src, T_lab_sen)
% Intersection of line and plane
Pa = T_lab_src(1:3,4); % Center of source coordinate system
%Pb = T_lab_src * [0;0;1;1]; % Point located on source z-axis
Pb = T_lab_d(1:3,4); % Target point
P0 = T_lab_sen(1:3,4); % Center of sensor coordinate system
P1 = T_lab_sen * [1;0;0;1]; % Point located on sensor x-axis
P2 = T_lab_sen * [0;1;0;1]; % Point located on sensor y-axis
Pint = IntersectLineAndPlane(Pa,Pb,P0,P1,P2); % Intersection point

% Record the intersection point
T_sen_lab = invTMatrix(T_lab_sen);
cpInSensorFrame = T_sen_lab * [Pint; 1];

% 0.5714 is used so that 1 cm cartesiam error = 1 deg error
cost = (0.5714*norm(cpInSensorFrame(1:3,1)))^2 + (acos(dot(T_lab_sen(1:3,3),T_lab_d(1:3,3))))^2;


function pnt = IntersectLineAndPlane(Pa,Pb,P0,P1,P2)
A = [(Pa(1:3,1) - Pb(1:3,1)), (P1(1:3,1) - P0(1:3,1)), (P2(1:3,1) - P0(1:3,1))];
b = Pa(1:3,1) - P0(1:3,1);
x = inv(A)*b;
pnt = Pa(1:3,1) + (Pb(1:3,1) - Pa(1:3,1))*x(1);

% -------------------------------------------------------------------------
%   invTMatrix
% -------------------------------------------------------------------------
function T_B_A = invTMatrix(T_A_B)
R_B_A = T_A_B(1:3,1:3)';
T_B_A = [R_B_A, -R_B_A*T_A_B(1:3,4); 0, 0, 0, 1];