function [Ni,We] = planar_local_planner(Ne, Nr, Prob)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
t0 = Ne(1); x0 = Ne(2:end);
tf = Nr(1); xf = Nr(2:end);



[U,Path] = connect_min_effort(to, Xo, tf, Xf, N, @planar_robots_ode);

branch.Path = Path;
branch.control = U;



function [q,qp] = map_task_2_joint_space(X, R_w_0, R_0_3)
P_w0_t = X(1:2);
V_t = [X(3:4); 0];


if (~isempty(q))
    % Map v -> qp
    zhat = [0;0;1]; % Planar robot so all joint axes point in pos z-direction

    % Rotation matricies
    R_0_1 = rotz(q(1));
    R_1_2 = rotz(q(2));
    %R_2_3 = rotz(q(3));

    % Constant joint to joint position vectors for the planar PA-10
    P_r0_j1 = [l0;0;0]; % .317
    P_j1_j2 = [l1;0;0]; %.45
    P_j2_j3 = [l2;0;0]; % .48

    % Position vectors to locate joints in world coordinate system
    P_w0_j1 = P_w0_r0 + R_w_0 * P_r0_j1;
    P_w0_j2 = P_w0_j1 + R_w_0 * R_0_1 * P_j1_j2;
    P_w0_j3 = P_w0_j2 + R_w_0 * R_0_1 * R_1_2 * P_j2_j3;

    % Position vectors to locate target point in relation to joint centers
    P_j1_t = P_w0_t - P_w0_j1;
    P_j2_t = P_w0_t - P_w0_j2;
    P_j3_t = P_w0_t - P_w0_j3;

    % Linear velocity jacobian matrix
    Jv = [cross(zhat,P_j1_t), cross(zhat,P_j2_t), cross(zhat,P_j3_t)];

    % Rotation velocity jacobian matrix
    %Jw = repmat(zhat,1,3);
    Jw = [];

    % Total jacobian and its generalized inverse
    J = [Jv; Jw];
    Jinv = J' * pinv( J * J' );
    
    % Compute joint velocities
    qp = Jinv * V_t;
else
    qp = [];
end