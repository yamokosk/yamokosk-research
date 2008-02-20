function J = cost_min_effort(y,prob)
% Unpack data
t0 = Prob.user.t0;
tf = Prob.user.tf;
nc = Prob.user.nc;
N = Prob.user.N;
W = Prob.user.W;
U = reshape(y(1:nc*N),nc,N);

% Compute sum_k( w(k) * G(X(tk), U(tk), tk) )
Gk = sum(U .^ 2);
sum_k_WkGk = Gk * W;
J = (tf - t0)/2 * sum_k_WkGk;