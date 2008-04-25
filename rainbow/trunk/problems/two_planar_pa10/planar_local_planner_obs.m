function [branch,exitflag,exitmsg] = planar_local_planner_obs(X0, Xf, Prob)
% Local planner for 3DOF planar robots
%
% SYNTAX
%
%   branch = planar_local_planner_obs(X0, Xf, Prob)
%
% DESCRIPTION
%
%   Attempts to connect states X0 and Xf with an admissible path.

% X0 and Xf are generic representations of the problems state space. Unpack
% the major variables to make it more human readable.
t0 = X0(1); x0 = X0(2:end);
tf = Xf(1); xf = Xf(2:end);

q_src_0 = x0(1:3); q_src_f = xf(1:3);   % Source robot's joint angles
qp_src_0 = x0(4:6); qp_src_f = xf(4:6); % Source robot's joint velocities

q_sen_0 = x0(7:9); q_sen_f = xf(7:9);   % Sensor robot's joint angles
qp_sen_0 = x0(10:12); qp_sen_f = xf(10:12); % Sensor robot's joing velocities

% Create output structure and variables
branch = struct('states', [], 'control', [], 'score', NaN);
exitflag = -1; exitmsg = '';

%% Check #1: Mean velocity can't exceed robot's joint velocity limits
qp_src_bar = (q_src_f - q_src_0) ./ (tf - t0);
src_check = find( (qp_src_bar > Prob.x_ub(5:7)) || (qp_src_bar < Prob.x_lb(5:7)) );
if ( ~isempty( src_check ) )
    exitflag = -1; exitmsg = ['Required mean velocity exceeded source robots joint limits'];
    return;
end

qp_sen_bar = (q_sen_f - q_sen_0) ./ (tf - t0);
src_check = find( (qp_sen_bar > Prob.x_ub(5:7)) || (qp_sen_bar < Prob.x_lb(5:7)) );
if ( ~isempty( src_check ) )
    exitflag = -1; exitmsg = ['Required mean velocity exceeded sensor robots joint limits'];
    return;
end

%% Check #2: Sign of q0 and qf should be the same
src_check = find( sign(q_src_f - q_src_0) ~= sign(qp_src_f - qp_src_0) );
if ( ~isempty( src_check ) )
    exitflag = -1; exitmsg = ['Final state did not fall in quadrant 1 or 3 for source robot.'];
    return;
end

sen_check = find( sign(q_sen_f - q_sen_0) ~= sign(qp_sen_f - qp_sen_0) );
if ( ~isempty( src_check ) )
    exitflag = -1; exitmsg = ['Final state did not fall in quadrant 1 or 3 for sensor robot.'];
    return;
end

%% Attempt to connect initial and final states
UseGPOCS = false;
if ( isfield(Prob.userdata, 'UseGPOCS') )
    UseGPOCS = Prob.userdata.UseGPOCS;
end

T = []; U = []; X = [];
t_span = [t0, tf];
N = 10;
if (UseGPOCS)
    [T, U, X, exitflag, exitmsg] = ...
         connect_min_effort(tspan, x0, xf, Prob.x_lb(2:end), Prob.x_ub(2:end), ...
         Prob.u_lb, Prob.u_ub, N, Prob.odefun, @planar_cost);
else
    [T, U, X, exitflag, exitmsg] = ...
         connect_shortest_dist(tspan, x0, xf, Prob.x_lb(2:end), Prob.x_ub(2:end), ...
         Prob.u_lb, Prob.u_ub, N, Prob.odefun, @planar_cost);
end

branch.states = [T; X];
branch.control = U;

% %% Check branch for collisions if the option is specified
% CheckCollisions = false;
% if ( isfield(Prob.userdata, 'CheckCollisions') )
%     CheckCollisions = Prob.userdata.CheckCollisions;
% end
% 
% if (CheckCollisions)
%     % Currently SceneML is the only collision
% names = {'q_src_2'; 'q_src_3'; 'q_src_5'; 'q_sen_2'; 'q_sen_3'; 'q_sen_5'};
% Ni = [];
% We = [];
% exitflag = 1;
% exitmsg = ['No suitable path could be found.'];
% for n = 1:size(X,2)
%     qsrc = X(2:4,n); qsen = X(8:10,n);
%     sceneSetVars(names, [qsrc; qsen]);
%     if (sceneCollisionState)
%         break;
%     else
%         Ni(:,n) = [T(n); X(:,n)];
%         We(:,n) = 0;
%         exitflag = 0;
%         exitmsg = '';
%     end    
% end
% 
% 
% 
% function U_cost = compute_control_cost(U, lb, ub)
% [N, nu] = size(U);
% U_cost = zeros(N,1);
% 
% % First normalize U
% for n = 1:N
%     U_normalized = (2*U(n,:) - (ub + lb)) ./ (ub - lb);
%     U_cost(n) = sqrt(U_normalized * U_normalized');
% end