function J = cost_min_effort(y,Prob)
% Unpack data
t0 = Prob.user.t0;
tf = Prob.user.tf;
nu = Prob.user.nu;
N = Prob.user.N;
w = Prob.user.w;
U = reshape(y(1:nu*N),nu,N);

% Compute sum_k( w(k) * G(X(tk), U(tk), tk) )
Gk = sum(U .^ 2);
sum_k_WkGk = Gk * w;
J = (tf - t0)/2 * sum_k_WkGk;