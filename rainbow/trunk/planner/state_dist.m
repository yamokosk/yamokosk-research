function [reached, dq, dqp] = state_dist(x,x_goal)
eps_dq = 0.5; eps_dqp = 10;

q_diff = x(1:6,1) - x_goal(1:6,1);
dq = sqrt( q_diff' * q_diff );

% qp_diff = x(7:12,1) - x_goal(7:12,1);
% dqp = sqrt( qp_diff' * qp_diff );
dqp = 0;

reached = false;
% if ( (dq < eps_dq) && (dqp < eps_dqp) )
%     reached = true;
% end

if ( (dq < eps_dq) )
    reached = true;
end