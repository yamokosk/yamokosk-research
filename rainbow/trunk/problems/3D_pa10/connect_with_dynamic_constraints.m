function xf_out = connect_with_dynamic_constraints(t0, tf, x0, xf, xlb, xub, ulb, uub, robot_model, testing)
% Local planner for PA-10 robots

% Local variables
q0 = x0(1:6,1);
qp0 = x0(7:12,1);
qf = xf(1:6,1);
qpf = xf(7:12,1);

num_time_frames = 12;
Ts = (tf - t0)/(num_time_frames - 1);
timevec = linspace(t0,tf,num_time_frames)';

% Create b-spline matrices
[N, Np, Npp, num_b_nodes] = create_bspline_matrices(Ts, num_time_frames);

% Matrix use to normalize difference calculations during optimization
R = diag( (xub - xlb).^2 );
R = inv(R);

% Create guess
guess = create_guess(num_b_nodes, q0, qf);

% % Call the lsqnonlin function.
% default_opts = optimset();
% my_opts = optimset(default_opts, 'Jacobian', 'off', ...  % Tell Matlab we are taking care of the Jacobian calculation.
%                                  'Display', 'off');     % No display.
% warning off all;
% opt_nodes = lsqnonlin(@mycost, packed_guess);

% Call the fminmax function
default_opts = optimset();
myopts = optimset(default_opts, 'GradObj',			'on', ...		% User provided gradient of cost function?
								'GradConstr',		'off', ...		% User provided gradient of constraint function?
								'DerivativeCheck',	'on', ...	% Good idea to set to 'on' only to test user supplied functions
                                'Display',			'iter');	% Algorithm display properties
%warning off all;
opt_nodes = fmincon(@mycost, guess, [], [], [], [], [], [], @mycon, myopts);

% unpack the answer
unpacked_opt_nodes = reshape(opt_nodes, num_b_nodes, 6);

q_opt = N * unpacked_opt_nodes;
qp_opt = Np * unpacked_opt_nodes;
qpp_opt = Npp * unpacked_opt_nodes;
tau_opt = zeros(num_time_frames, 6);
for t = 1:num_time_frames
    tau_opt(t,:) = torque(robot_model, q_opt(t,:)', qp_opt(t,:)', qpp_opt(t,:)')';
end

if (testing)
	t = linspace(t0,tf,num_time_frames)';
	close all;
	h = figure(1);
	for n = 1:6
		subplot(3,2,n);
		plot(t,tau_opt(:,n));
		line([t0; tf], [ulb(n); ulb(n)]);
		line([t0; tf], [uub(n); uub(n)]);
		title(['\tau_' num2str(n)]);
	end
	
	h = figure(2);
	for n = 1:6
		subplot(3,2,n);
		hold on;
		plot(tf,qf(n),'*');
		plot(t,q_opt(:,n));
		title(['q_' num2str(n)]);
	end
end

xf_out = [q_opt(end,:)'; qp_opt(end,:)'];

	function [c, ceq, gc, gceq] = mycon(packed_nodes)
		% Unpack nodes
        unpacked_nodes = reshape(packed_nodes, num_b_nodes, 6);
        
        % Generate trajectories from B-Spline nodes
        q = N * unpacked_nodes;
        qp = Np * unpacked_nodes;
        qpp = Npp * unpacked_nodes;
        
        % Calculate joint torques
		C = zeros(num_time_frames,6*2); 
        for t = 1:num_time_frames
            tau = torque(robot_model, q(t,1:6)', qp(t,1:6)', qpp(t,1:6)')';
            
			C(t,1:6) = tau - uub';
			C(t,7:12) = ulb' - tau;
		end
		
		% Inequaltiy constraint on the joint torques
		c = reshape(C, num_time_frames*6*2, 1);
		
		% Equality constraint on the initial conditions
		d0 = q(1,1:6)' - q0;
		dp0 = qp(1,1:6)' - qp0;
		ceq = [d0; dp0];
		
		if (nargout > 2)
			% Compute gradients is necessary
			nc = length(c);
			nceq = length(ceq);
			nx = length(packed_nodes);
			gc = zeros(nx,nc);
			
			% gceq - Nceq x Nx
			Z = zeros(1,num_b_nodes);
			A = N(1,:);
			gceq1 =  [ A, Z, Z, Z, Z, Z; ...
				       Z, A, Z, Z, Z, Z; ...
					   Z, Z, A, Z, Z, Z; ...
					   Z, Z, Z, A, Z, Z; ...
					   Z, Z, Z, Z, A, Z; ...
					   Z, Z, Z, Z, Z, A]';
			A = Np(1,:);
			gceq2 = [ A, Z, Z, Z, Z, Z; ...
				      Z, A, Z, Z, Z, Z; ...
					  Z, Z, A, Z, Z, Z; ...
					  Z, Z, Z, A, Z, Z; ...
					  Z, Z, Z, Z, A, Z; ...
					  Z, Z, Z, Z, Z, A]';
			gceq = sparse( [gceq1, gceq2] );
		end
	end

%     function [F, G] = mycost(packed_nodes)
%         % Unpack nodes
%         unpacked_nodes = reshape(packed_nodes, num_b_nodes, 6);
%         
%         % Generate trajectories from B-Spline nodes
%         q = N * unpacked_nodes;
%         qp = Np * unpacked_nodes;
%         
% 		% Compute cost
%         df = (q(end,1:6)' - qf) ./ (xub(1:6) - xlb(1:6));
%         dpf = (qp(end,1:6)' - qpf) ./ (xub(7:12) - xlb(7:12));
%         
% 		% Might need a penalty on accelerations
%         F = [df; dpf];
% 
% 		if (nargout > 1)
% 			Z = zeros(1,num_b_nodes);
% 			A = N(end,:);
% 			g1 = [ A, Z, Z, Z, Z, Z; ...
% 				   Z, A, Z, Z, Z, Z; ...
% 				   Z, Z, A, Z, Z, Z; ...
% 				   Z, Z, Z, A, Z, Z; ...
% 				   Z, Z, Z, Z, A, Z; ...
% 				   Z, Z, Z, Z, Z, A]';
% 			A = Np(end,:);
% 			g2 = [ A, Z, Z, Z, Z, Z; ...
% 				   Z, A, Z, Z, Z, Z; ...
% 				   Z, Z, A, Z, Z, Z; ...
% 				   Z, Z, Z, A, Z, Z; ...
% 				   Z, Z, Z, Z, A, Z; ...
% 				   Z, Z, Z, Z, Z, A]';
% 			G = sparse( [g1, g2] );
% 		end
% 	end

	function [F, G] = mycost(packed_nodes)
        % Unpack nodes
        unpacked_nodes = reshape(packed_nodes, num_b_nodes, 6);
        
        % Generate trajectories from B-Spline nodes
        q = N * unpacked_nodes;
        qp = Np * unpacked_nodes;
        
		% Compute end point cost
        df = q(end,:)' - qf;
        dpf = qp(end,:)' - qpf;
        d = [df; dpf];
		
		% Compute path cost
		L = zeros(1,num_time_frames);
		P = eye(6,6);
		for t = 1:num_time_frames
			L(1,t) = q(t,:) * P * q(t,:)';
		end
		
		
		% Might need a penalty on accelerations
        F = d' * R * d + trapz(timevec, L);

		if (nargout > 1)
			Z = zeros(1,num_b_nodes);
			A = N(end,:);
			g1 = [ A, Z, Z, Z, Z, Z; ...
				   Z, A, Z, Z, Z, Z; ...
				   Z, Z, A, Z, Z, Z; ...
				   Z, Z, Z, A, Z, Z; ...
				   Z, Z, Z, Z, A, Z; ...
				   Z, Z, Z, Z, Z, A];
			A = Np(end,:);
			g2 = [ A, Z, Z, Z, Z, Z; ...
				   Z, A, Z, Z, Z, Z; ...
				   Z, Z, A, Z, Z, Z; ...
				   Z, Z, Z, A, Z, Z; ...
				   Z, Z, Z, Z, A, Z; ...
				   Z, Z, Z, Z, Z, A];
			g = [g1; g2];
			G = 2 * g' * R * d;
			
			L = [];
			for t = 1:num_time_frames
				A = N(t,:);
				g3 = [ A, Z, Z, Z, Z, Z; ...
					   Z, A, Z, Z, Z, Z; ...
					   Z, Z, A, Z, Z, Z; ...
					   Z, Z, Z, A, Z, Z; ...
					   Z, Z, Z, Z, A, Z; ...
					   Z, Z, Z, Z, Z, A];
				g4 = [ g3' * P * q(t,:)' ]';
				L = [L; g4];
			end
			G = G + 2*trapz(timevec,L)';
		end
	end
end

% Creates the b-spline matrices
function [N, Np, Npp, num_b_nodes] = create_bspline_matrices(Ts, num_time_frames)
order = 5;
num_b_nodes = 6;
[N, Np, Npp] = bspline_creator(order, num_b_nodes, num_time_frames, Ts);
end

% Function to create a simple guess
function packed_guess = create_guess(num_b_nodes, q0, qf)
alpha = linspace(0,1,num_b_nodes);
q_guess = zeros(num_b_nodes, length(q0));
for n = 1:num_b_nodes
    q_guess(n,:) = [(qf - q0)*alpha(n) + q0]';
end

packed_guess = reshape(q_guess, 6*num_b_nodes, 1);
end
