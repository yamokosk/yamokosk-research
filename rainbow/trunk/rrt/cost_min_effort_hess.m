function dGdy = cost_min_effort_hess(y,Prob)
% Unpack data
tau = Prob.user.tau;
t0 = Prob.user.t0;
tf = Prob.user.tf;
nc = Prob.user.nc;
nu = Prob.user.nu;
ns = Prob.user.ns;
N = Prob.user.N;
w = Prob.user.w;
U = reshape(y(1:nu*N),nu,N);

% Compute dJ2/d2U
W_mat = repmat(w',nu,1);
W_diag = diag(reshape(W_mat,nu*N,1));

dGdy = sparse((nu+ns)*N, (nu+ns)*N);
dGdy(1:nu*N,1:nu*N) = (tf - t0) * sparse(W_diag);