function [J,Jinv] = jacobian_planar_pa10(q, rs)
% Map v -> qp
zhat = [0;0;1]; % Planar robot so all joint axes point in pos z-direction

% Rotation matricies
[T_w_t, T_w_1, T_w_2, T_w_3] = fkine_planar_pa10(q, rs);
R_w_1 = T_w_1(1:3,1:3);
R_w_2 = T_w_2(1:3,1:3);
R_w_3 = T_w_3(1:3,1:3);

% Constant joint to joint position vectors for the planar PA-10
P1_j1_j2 = [rs.l1;0;0;1]; %.45
P2_j2_j3 = [rs.l2;0;0;1]; %.48
P3_j3_t  = [rs.l3;0;0;1];

% Position vectors to locate target point in relation to joint centers
Pw_j3_t = R_w_3 * P3_j3_t(1:3,1);
Pw_j2_t = R_w_2 * P2_j2_j3(1:3,1) + Pw_j3_t;
Pw_j1_t = R_w_1 * P1_j1_j2(1:3,1) + Pw_j2_t;

% Linear velocity jacobian matrix
Jv = [cross(zhat,Pw_j1_t(1:3)), cross(zhat,Pw_j2_t(1:3)), cross(zhat,Pw_j3_t(1:3))];

% Rotation velocity jacobian matrix
Jw = [];

% Total jacobian and its generalized inverse
J = [Jv(1:2,:); Jw];

if nargout > 1
    Jinv = J' * pinv( J * J' );
    %Jinv = pinv(J);
end