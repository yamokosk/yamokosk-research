function path = constrained_straight_line_planner(X0, Xf, udata)
% Local planner for 3DOF planar robots
%gr = (1 + sqrt(5))/2 - 1;   % golden ratio
gr = 0.5;

Xi = [];
for n = 0:10
    Xi = X0 + gr^n * (Xf - X0);
    Xi(end) = Xf(end);
    [valid, x_src_0, x_src_i, x_sen_0, x_sen_i] = checkBoundaryConditions(X0, Xi, udata);
    if ( valid )
        break;
    end
end

if ( ~isempty(Xi) )
    d_src = x_src_i(1:6,1) - x_src_0(1:6,1);
    d_sen = x_sen_i(1:6,1) - x_sen_0(1:6,1);
    dist = sqrt( d_src' * d_src + d_sen' * d_sen );
    path = struct('xi',[X0, Xi],'ew',dist);
else
    path = struct('x1',[],'ew',[]);
end

end

function [valid, x_src_0, x_src_f, x_sen_0, x_sen_f] = checkBoundaryConditions(X0, Xf, udata)

valid = false; 
x_src_0 = []; x_src_f = [];
x_sen_0 = []; x_sen_f = [];

% Pull out time
t0 = X0(end);
tf = Xf(end);
dt = tf - t0;

% Map initial to robot states. Note, we do not need to check whether the
% mapping is good as X0 is an pre-existing node in the tree.. it should
% have been checked for goodness before being put into the tree.
[x_src_0, x_sen_0] = metricToJointSpaceMap(X0, udata);

% Map final to robot states. We need to check here to make sure the mapping
% was successful.
[x_src_f, x_sen_f] = metricToJointSpaceMap(Xf, udata);
if ( isempty(x_src_f) || isempty(x_sen_f) )
    return;
end

% Check that we are not violating velocity contraints on the robots
q_src_0 = x_src_0(1:6,1); q_src_f = x_src_f(1:6,1); % Source robot's joint angles
q_sen_0 = x_sen_0(1:6,1); q_sen_f = x_sen_f(1:6,1); % Sensor robot's joint angles

% Check #1: Mean velocity can't exceed robot's joint velocity limits
qp_src_bar = (q_src_f - q_src_0) ./ dt;
src_check = find( abs(qp_src_bar) > udata.rsrc.qpmax' );
if ( ~isempty( src_check ) )
    return;
end

qp_sen_bar = (q_sen_f - q_sen_0) ./ dt;
sen_check = find( abs(qp_sen_bar) > udata.rsen.qpmax' );
if ( ~isempty( sen_check ) )
    return;
end

% % Check #2: Sign of q0 and qf should be the same
% src_check = find( sign(q_src_f - q_src_0) ~= sign(qp_src_f - qp_src_0) );
% if ( ~isempty( src_check ) )
%     return;
% end
% 
% sen_check = find( sign(q_sen_f - q_sen_0) ~= sign(qp_sen_f - qp_sen_0) );
% if ( ~isempty( sen_check ) )
%     return;
% end

valid = true;

end