function [Ni,We,exitflag,exitmsg] = planar_local_planner_obs(Ne, Nr, Prob)
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

% if ( ~isempty(src_check) || ~isempty(sen_check) )
%     exitflag = 1;
%     exitmsg = ['Avg joint velocity exceeded joint limits'];
%     Ni = [];
%     We = [];
%     return;
% end

% Attempt to connect states
% [T, U, X, exitflag, exitmsg] = ...
%      connect_min_effort(tspan, x0, xf, Prob.x_lb(2:end), Prob.x_ub(2:end), ...
%      Prob.u_lb, Prob.u_ub, N, Prob.odefun, @planar_cost);

[T, U, X, exitflag, exitmsg] = ...
     connect_shortest_dist(tspan, x0, xf, Prob.x_lb(2:end), Prob.x_ub(2:end), ...
     Prob.u_lb, Prob.u_ub, N, Prob.odefun, @planar_cost);
 
names = {'q_src_2'; 'q_src_3'; 'q_src_5'; 'q_sen_2'; 'q_sen_3'; 'q_sen_5'};
Ni = [];
We = [];
exitflag = 1;
exitmsg = ['No suitable path could be found.'];
for n = 1:size(X,2)
    qsrc = X(2:4,n); qsen = X(8:10,n);
    sceneSetVars(names, [qsrc; qsen]);
    if (sceneCollisionState)
        break;
    else
        Ni(:,n) = [T(n); X(:,n)];
        We(:,n) = 0;
        exitflag = 0;
        exitmsg = '';
    end    
end



function U_cost = compute_control_cost(U, lb, ub)
[N, nu] = size(U);
U_cost = zeros(N,1);

% First normalize U
for n = 1:N
    U_normalized = (2*U(n,:) - (ub + lb)) ./ (ub - lb);
    U_cost(n) = sqrt(U_normalized * U_normalized');
end