function path = lp2(X0, Xf, udata)
% Local planner for 3DOF planar robots

[valid, t0, tf, ...
    q_src_0, qp_src_0, q_src_f, qp_src_f, ...
    q_sen_0, qp_sen_0, q_sen_f, qp_sen_f] = checkBoundaryConditions(X0, Xf, udata.x_lb, udata.x_ub);

if ( ~valid )
    path = struct('xi',[],'ew',[]);
    return;
end

order = 5;
num_b_nodes = 6;
num_time_frames = 12;

alpha = linspace(0,1,num_time_frames);
time = (tf - t0) * alpha + t0;
Ts = time(2)-time(1);

[N, Np, Npp] = bspline_creator(order, num_b_nodes, num_time_frames, Ts);

alpha = linspace(0,1,num_b_nodes);
X_guess = zeros(num_b_nodes, length(X0));
for n = 1:num_b_nodes
    X_guess(n,:) = [(Xf - X0)*alpha(n) + X0]';
end

opt_guess = [X_guess(:,1:3), X_guess(:,7:9)];
packed_guess = reshape(opt_guess, 6*num_b_nodes, 1);

% Call the lsqnonlin function.
default_opts = optimset();
my_opts = optimset(default_opts, 'Jacobian', 'off', ...  % Tell Matlab we are taking care of the Jacobian calculation.
                                 'Display', 'off');     % No display.
warning off all;
opt_nodes = lsqnonlin(@mycost, packed_guess);
unpacked_opt_nodes = reshape(opt_nodes, num_b_nodes, 6);

q_opt = N * unpacked_opt_nodes;
qp_opt = Np * unpacked_opt_nodes;
qpp_opt = Npp * unpacked_opt_nodes;
Tau_opt = zeros(num_time_frames, 6);
for t = 1:num_time_frames
    Tau_opt(t,1:3) = torque(udata.rsrc, q_opt(t,1:3)', qp_opt(t,1:3)', qpp_opt(t,1:3)')';
    Tau_opt(t,4:6) = torque(udata.rsen, q_opt(t,4:6)', qp_opt(t,4:6)', qpp_opt(t,4:6)')';
end

tau_rms = sqrt( diag( Tau_opt * Tau_opt' ) );
X = [q_opt(:,1:3)'; qp_opt(:,1:3)'; q_opt(:,4:6)'; qp_opt(:,4:6)'; time];
path = struct('xi',X,'ew',tau_rms(1:end-1));

    function fval = mycost(packed_nodes)
        % Unpack nodes
        unpacked_nodes = reshape(packed_nodes, num_b_nodes, 6);
        
        % Generate trajectories from B-Spline nodes
        q = N * unpacked_nodes;
        qp = Np * unpacked_nodes;
        qpp = Npp * unpacked_nodes;
        
        % Calculate joint torques
        Tau = zeros(num_time_frames, 6);
        for t = 1:num_time_frames
            Tau(t,1:3) = torque(udata.rsrc, q(t,1:3)', qp(t,1:3)', qpp(t,1:3)')';
            Tau(t,4:6) = torque(udata.rsen, q(t,4:6)', qp(t,4:6)', qpp(t,4:6)')';
        end

        tau_rms = sqrt( diag( Tau' * Tau ) );
        tau_score = tau_rms ./ (udata.u_ub - udata.u_lb);
        
        q_src_icdiff = (q(1,1:3)' - q_src_0) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        q_sen_icdiff = (q(1,4:6)' - q_sen_0) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        qp_src_icdiff = (qp(1,1:3)' - qp_src_0) ./ (udata.x_ub(4:6) - udata.x_lb(4:6));
        qp_sen_icdiff = (qp(1,4:6)' - qp_sen_0) ./ (udata.x_ub(4:6) - udata.x_lb(4:6));
        
        q_src_fcdiff = (q(end,1:3)' - q_src_f) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        q_sen_fcdiff = (q(end,4:6)' - q_sen_f) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        qp_src_fcdiff = (qp(end,1:3)' - qp_src_f) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        qp_sen_fcdiff = (qp(end,4:6)' - qp_sen_f) ./ (udata.x_ub(1:3) - udata.x_lb(1:3));
        
        fval = [tau_score; q_src_icdiff; q_sen_icdiff; qp_src_icdiff; qp_sen_icdiff; ...
                q_src_fcdiff; q_sen_fcdiff; qp_src_fcdiff; qp_sen_fcdiff];
        
%         if nargout > 1  % Two output arguments
%             
%             for r = 1:udata.num_b_nodes
%                 
%             end
%             for n = 1:length(packed_nodes)
%                 da = unpacked_nodes(:,n)
%                 dq = udata.N * unpacked_nodes(:,n);
%                 dqp = udata.Np * unpacked_nodes(:,n);
%                 dqpp = udata.Npp * unpacked_nodes(:,n);
%             
%                 
%             end
% 
%             for t = 1:udata.num_time_frames
%                 [tau, dtau] = delta_torque(q(t,1:3)', qp(t,1:3)', qpp(t,1:3)', dq, dqp, dqpp);
%             end
%             
%             % Jacobian of the function evaluate at x
%             J = [];
%         end
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