function path = lp(X0, Xf, udata)
% Local planner for 3DOF planar robots

[valid, t0, tf, ...
    q_src_0, qp_src_0, q_src_f, qp_src_f, ...
    q_sen_0, qp_sen_0, q_sen_f, qp_sen_f] = checkBoundaryConditions(X0, Xf, udata.x_lb, udata.x_ub);

if ( ~valid )
    path = struct('xi',[],'ew',[]);
    return;
end

% Attempt to connect initial and final states
x0 = X0(1:end-1,1);
xf = Xf(1:end-1,1);
t_span = [t0, tf];
N = 8;
[T, U, X, exitflag, exitmsg] = ...
     connect_min_effort(t_span, x0, xf, udata.x_lb(1:end-1), udata.x_ub(1:end-1), udata.u_lb, udata.u_ub, N, udata);

if (exitflag == 0)
    u_squared = U * U';
    tau_rms = sqrt( diag(u_squared) );
    path = struct('xi',[X'; T'],'ew',tau_rms(1:end-1));
else
    disp(exitmsg);
    path = struct('xi',[],'ew',[]);
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