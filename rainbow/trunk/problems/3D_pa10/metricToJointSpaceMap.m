function [x_src, x_sen] = metricToJointSpaceMap(state, udata)

src_EE = state(1:12,1);
d = state(13,1);
t = state(14,1);
qp_max = udata.rsrc.qpmax';

% Create source robot T-matrix
T_src = rotk(src_EE(4:6,1));
T_src(1:3,4) = src_EE(1:3,1);
x_src = computeRobotState(udata.rsrc, T_src, src_EE(7:12,1), qp_max);

% Create sensor robot T-matrix
T_d = eye(4);
T_d(3,4) = d;
T_sen = T_src * transl(0,0,d) * roty(pi);
x_sen = computeRobotState(udata.rsen, T_sen, src_EE(7:12,1), qp_max);


function [x, converged] = computeRobotState(model, T_world_EE, vd, qp_limit)

% Use inverse kinematics to compute joint angles
[q, converged] = inv_kin(model, T_world_EE);
if (~converged)
    x = [];
    return;
end

J_inv = pinv( jacobian(model, q) );
qp = J_inv * vd;

ind = find( abs(qp) > qp_limit );

if ( ~isempty(ind) )
    s = norm(vd);
    n = J_inv*(vd/s);
    s = min( qp_limit ./ abs(n) );
    
    qp = s*n;
end
x = [q; qp];