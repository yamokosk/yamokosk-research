function [Ni,We,exitflag,exitmsg] = planar_local_planner(Ne, Nr, Prob)
% Connect algorithm - finds time optimal path and consideres joint angle
% and torque limits
%t0 = Ne(1); x0_src = Ne(2:7); x0_sen = Ne(8:13);
%tf = Nr(1); xf_src = Nr(2:7); xf_sen = Nr(8:13);
t0 = Ne(1); x0 = Ne(2:end);
tf = Nr(1); xf = Nr(2:end);
qsrc0 = x0(1:3); qsrcf = xf(1:3);
qsen0 = x0(7:9); qsenf = xf(7:9);

tspan = [t0, tf];
N = 10;

Ni = [];
We = [];

% Reality check on states.. can't exceed robot capabilties
qpsrc = (qsrcf - qsrc0) ./ (tf - t0);
src_check = find( abs(qpsrc) > Prob.x_ub(5:7) );

qpsen = (qsenf - qsen0) ./ (tf - t0);
sen_check = find( abs(qpsen) > Prob.x_ub(5:7) );

if ( ~isempty(src_check) || ~isempty(sen_check) )
    exitflag = 1;
    exitmsg = ['Avg joint velocity exceeded joint limits'];
    Ni = [];
    We = [];
    return;
end

% % Attempt to connect states for source robot
% [T_src, U_src, X_src, exitflag, exitmsg] = ...
%     connect_min_effort(tspan, x0_src, xf_src, Prob.x_lb(2:7), Prob.x_ub(2:7), Prob.u_lb(1:3), Prob.u_ub(1:3), N, Prob.odefun, @planar_cost);
% 
% if ( exitflag ~= 0 )
%     exitmsg = ['Failed on source robot. ' exitmsg];
%     return;
% end
% 
% % Attempt to connect states for sensor robot
% [T_sen, U_sen, X_sen, exitflag, exitmsg] = ...
%     connect_min_effort(tspan, x0_sen, xf_sen, Prob.x_lb(2:7), Prob.x_ub(2:7), Prob.u_lb(1:3), Prob.u_ub(1:3), N, Prob.odefun, @planar_cost);
% 
% if ( exitflag ~= 0 )
%     exitmsg = ['Failed on sensor robot. ' exitmsg];
%     return;
% end

% Attempt to connect states
[T, U, X, exitflag, exitmsg] = ...
     connect_min_effort(tspan, x0, xf, Prob.x_lb(2:end), Prob.x_ub(2:end), ...
     Prob.u_lb, Prob.u_ub, N, Prob.odefun, @planar_cost);

% % Able to connect both robots to their states. Report results
% Ni = [T_src'; [X_src, X_sen]'];
% 
% % Compute "cost" associated with each control strategy
% U_src_cost = compute_control_cost(U_src, Prob.u_lb(1:3)', Prob.u_ub(1:3)');
% U_sen_cost = compute_control_cost(U_sen, Prob.u_lb(1:3)', Prob.u_ub(1:3)');
% We = U_src_cost + U_sen_cost;

Ni = [T'; X'];
We = compute_control_cost(U, Prob.u_lb', Prob.u_ub');


function U_cost = compute_control_cost(U, lb, ub)
[N, nu] = size(U);
U_cost = zeros(N,1);

% First normalize U
for n = 1:N
    U_normalized = (2*U(n,:) - (ub + lb)) ./ (ub - lb);
    U_cost(n) = sqrt(U_normalized * U_normalized');
end