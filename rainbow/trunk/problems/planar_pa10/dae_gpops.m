function dae = dae_gpops(sol)

% Pull out variables
t = sol.time;
x1 = sol.state(:,1:6);
x2 = sol.state(:,7:12);
u = sol.control;
N = length(t);  % Number of nodes

x1_dot = x2;
x2_dot = pa10_acceleration(u',x1',x2')';
dae = [x1_dot, x2_dot];

% Z = zeros(6,6);
% I = eye(6);
% 
% df1_dx1 = Z;
% df1_dx2 = I;
% df1_du = Z;
% df1_dt = zeros(size(t));
% df2_dt = zeros(size(t));

% dae = zeros(N,12);
% derivdae = zeros(12*N,19);
% j = 1:N:N*N;
% for k = 1:N;
%     % With gravity
%     Minv = pinv( inertia(rbt, x1(k,:)') );
%     C = coriolis(rbt, x1(k,:)', x2(k,:)');
%     G = gravity(rbt, x1(k,:)');
%     x1dot = x2;
%     x2dot = Minv * ( u(k,:)' - C - G );
%     
%     % Without gravity
%     Minv = pinv( inertia(rbt, x1(k,:)') );
%     C = coriolis(rbt, x1(k,:)', x2(k,:)');
%     x1dot = x2;
%     x2dot = Minv * ( u(k,:) - C );
% 
%     dae(k,:) = [x1dot', x2dot'];
%     
%     if (nargout > 1)
%         df2_dx1 = Z; % WRONG. Its complex and I need to figure it out.
%         df2_dx2 = Z; % WRONG. Its complex and I need to figure it out.
%         df2_du = Minv;
%      
%         derivdae(j(k):j(k)+N-1,:) = [df1_dx1, df1_dx2, df1_du, df1_dt; ...
%                                      df2_dx1, df2_dx2, df2_du, df2_dt];
%     end
% end
