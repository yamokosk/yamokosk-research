function dJdy = cost_min_effort_grad(y,Prob)
% Unpack data
tau = Prob.user.tau;
t0 = Prob.user.t0;
tf = Prob.user.tf;
nc = Prob.user.nc;
nu = Prob.user.nu;
ns = Prob.user.ns;
N = Prob.user.N;
w = Prob.user.w;
% U = reshape(y(1:nu*N),nu,N);

% % Compute U_dot = (U_k - U_k-1) / (t_k - t_k-1)
% U_km1 = circshift(U, [0 1]);
% U_km1(:,1) = zeros(nu,1);       % Maybe not good approximation.. for first column
% dU = U - U_km1;
% 
% t_k = ((tf - t0) * tau' + (tf - t0))/2;
% t_km1 = circshift(t_k, [0 1]);
% t_km1(1) = t0;
% dt = t_k - t_km1;
% 
% U_dot = dU ./ repmat(dt, nu, 1);

% Compute dJ/dU = (tf - t0) * w_k * U'k * U_dot
% w_mat = repmat(w',nu,1);
% dJdU = (tf - t0) * w_mat .* ( U .* U_dot );
U = y(1:nu*N);
w_mat = diag(reshape(repmat(w',nu,1),nu*N,1));
dJdU = (tf - t0) * w_mat * U;

% Compute dJ/dy
dJdy = [dJdU', zeros(1, ns*N)];
