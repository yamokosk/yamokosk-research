function [J,Jinv] = jacobian_planar_pa10(q)
l0 = .317;
l1 = .45;
l2 = .48;
l3 = .3;

% Map v -> qp
zhat = [0;0;1]; % Planar robot so all joint axes point in pos z-direction

% Rotation matricies
T_0_1 = transl(l0,0,0) * rotz(q(1));
T_1_2 = transl(l1,0,0) * rotz(q(2));
T_2_3 = transl(l2,0,0) * rotz(q(3));

T_0_2 = T_0_1 * T_1_2;
T_0_3 = T_0_2 * T_2_3;

% Constant joint to joint position vectors for the planar PA-10
%P0_r0_j1 = [l0;0;0;1]; % .317
P1_j1_j2 = [l1;0;0;1]; %.45
P2_j2_j3 = [l2;0;0;1]; % .48
P3_j3_t  = [l3;0;0;1];

% Position vectors to locate joints in world coordinate system
%P_w0_j1 = P_w0_r0 + R_w_0 * P_r0_j1;
%P_w0_j2 = P_w0_j1 + R_w_0 * R_0_1 * P_j1_j2;
%P_w0_j3 = P_w0_j2 + R_w_0 * R_0_1 * R_1_2 * P_j2_j3;

% Position vectors to locate target point in relation to joint centers
%P_j1_t = P_w0_t - P_w0_j1;
%P_j2_t = P_w0_t - P_w0_j2;
%P_j3_t = P_w0_t - P_w0_j3;

P0_j3_t = T_0_3 * P3_j3_t;
P0_j2_t = T_0_2 * P2_j2_j3 + P0_j3_t;
P0_j1_t = T_0_1 * P1_j1_j2 + P0_j2_t;

% Linear velocity jacobian matrix
Jv = [cross(zhat,P0_j1_t(1:3)), cross(zhat,P0_j2_t(1:3)), cross(zhat,P0_j3_t(1:3))];

% Rotation velocity jacobian matrix
%Jw = repmat(zhat,1,3);
Jw = [];

% Total jacobian and its generalized inverse
J = [Jv; Jw];

if nargout > 1
    Jinv = J' * pinv( J * J' );
end