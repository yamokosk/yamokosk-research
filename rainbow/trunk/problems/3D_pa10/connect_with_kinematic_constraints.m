function path = connect_with_kinematic_constraints(t0, tf, x0, xf, xlb, xub, ulb, uub, robot_model, testing)
% Local planner for 3DOF planar robots
%gr = (1 + sqrt(5))/2 - 1;   % golden ratio
gr = 0.5;

xi = [];
for n = 0:10
    xi = x0 + gr^n * (xf - x0);
    [valid, x_src_0, x_src_i, x_sen_0, x_sen_i] = checkBoundaryConditions(t0,tf,x0,xf,xlb,xub,robot_model);
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


function [valid, x_src_0, x_src_f, x_sen_0, x_sen_f] = checkBoundaryConditions(t0,tf,x0,xf,xlb,xub,robot_model)

valid = false; 
x_src_0 = []; x_src_f = [];
x_sen_0 = []; x_sen_f = [];

% Pull out time
dt = tf - t0;

% Map initial to robot states. Note, we do not need to check whether the
% mapping is good as X0 is an pre-existing node in the tree.. it should
% have been checked for goodness before being put into the tree.
[x_src_0, x_sen_0] = metricToJointSpaceMap(x0, xlb, xub, robot_model);

% Map final to robot states. We need to check here to make sure the mapping
% was successful.
[x_src_f, x_sen_f] = metricToJointSpaceMap(xf, xlb, xub, robot_model);
if ( isempty(x_src_f) || isempty(x_sen_f) )
    return;
end

% Check that we are not violating velocity contraints on the robots
q_src_0 = x_src_0(1:6,1); q_src_f = x_src_f(1:6,1); % Source robot's joint angles
q_sen_0 = x_sen_0(1:6,1); q_sen_f = x_sen_f(1:6,1); % Sensor robot's joint angles
qp_max = xub(7:12,1);

% Check #1: Mean velocity can't exceed robot's joint velocity limits
qp_src_bar = (q_src_f - q_src_0) ./ dt;
src_check = find( abs(qp_src_bar) > qp_max );
if ( ~isempty( src_check ) )
    return;
end

qp_sen_bar = (q_sen_f - q_sen_0) ./ dt;
sen_check = find( abs(qp_sen_bar) > qp_max );
if ( ~isempty( sen_check ) )
    return;
end

valid = true;

end