function path = constrained_straight_line_planner(X0, Xf, udata)
% Local planner for 3DOF planar robots

% Get robot states
[x_src_0, x_sen_0] = metricToJointSpaceMap(X0, udata);
[x_src_f, x_sen_f] = metricToJointSpaceMap(Xf, udata);
t0 = X0(end); tf = Xf(end);

if ( isempty(x_src_0) || isempty(x_src_f) || isempty(x_sen_0) || isempty(x_sen_f) )
    path = struct('xi',[],'ew',[]);
    return;
end

X_src_f = final_point(t0,tf,x_src_0,x_src_f);
X_sen_f = final_point(t0,tf,x_sen_0,x_sen_f);

if ( ~isempty(X_src_f) && ~isempty(X_sen_f) )
    % Put it all back together again
    xf = jointToMetricSpaceMap(X_src_f, X_sen_f, udata);

    d_src = X_src_f(1:6,1) - x_src_0(1:6,1);
    d_sen = X_sen_f(1:6,1) - x_sen_0(1:6,1);
    dist = sqrt( d_src' * d_src + d_sen' * d_sen );
    path = struct('xi',[X0, [xf; tf]],'ew',dist);
else
    path = struct('x1',[],'ew',[]);
end

end

function [valid, t0, tf, ...
    q_src_0, qp_src_0, q_src_f, qp_src_f, ...
    q_sen_0, qp_sen_0, q_sen_f, qp_sen_f] = checkBoundaryConditions(X0, Xf, x_lb, x_ub)

valid = false; 

% X0 and Xf are generic representations of the problems state space. Unpack
% the major variables to make it more human readable.
t0 = X0(end); x0 = X0(1:end-1);
tf = Xf(end); xf = Xf(1:end-1);

q_src_0 = x0(1:3); q_src_f = xf(1:3);   % Source robot's joint angles
qp_src_0 = x0(4:6); qp_src_f = xf(4:6); % Source robot's joint velocities

q_sen_0 = x0(7:9); q_sen_f = xf(7:9);   % Sensor robot's joint angles
qp_sen_0 = x0(10:12); qp_sen_f = xf(10:12); % Sensor robot's joing velocities

% Check #1: Mean velocity can't exceed robot's joint velocity limits
qp_src_bar = (q_src_f - q_src_0) ./ (tf - t0);
src_check = find( (qp_src_bar > x_ub(4:6)) | (qp_src_bar < x_lb(4:6)) );
if ( ~isempty( src_check ) )
    return;
end

qp_sen_bar = (q_sen_f - q_sen_0) ./ (tf - t0);
sen_check = find( (qp_sen_bar > x_ub(10:12)) | (qp_sen_bar < x_lb(10:12)) );
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