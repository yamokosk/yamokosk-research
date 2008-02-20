% Constraints function
function dc = con_min_tf_grad(z,Prob)
t0 = Prob.user.t0;
x0 = Prob.user.x0;
xf = Prob.user.xf;
dim = Prob.user.dim;
N = length(Prob.user.w);
nstates = dim*2;

% Unpack the data
[tf,U,X] = unpack(z,dim,N);

% Scale diff matrices by time scale
Db = Prob.user.Dbar;
D = Prob.user.D;
w = Prob.user.w;

% Allocate space for constraints
fi = zeros(nstates,N);
df_du = zeros(nstates, dim, N);
df_dx = zeros(nstates, nstates, N);

dc = zeros(nstates * (N+1), length(z));
dcf_du = zeros(nstates, dim, N);
dcf_dx = zeros(nstates, nstates, N);

% Compute constraints and its partial derivatives
for i = 1:N
    % Compute RHS of ODE at Xi, Ui, ti
    fi(:,i) = Prob.user.odefun(X(:,i),U(:,i));

    % Compute sum_k ( D(i,k) * X(tk) )
    Dik_mat = repmat(D(i,:), nstates, 1);
    sum_k_DikXk = sum(Dik_mat .* X, 2);
    
    % Compute partial of ith dynamic constraint w.r.t. the final time: 
    %   dc(i)/dtf = -2/(tf - t0)^2 * ( D_bar(i) * X(t0) + sum_k ( D(i,k) * X(tk) )
    dc_dtf = -2/(tf - t0)^2 * ( Db(i)*x0 + sum_k_DikXk );
    
    % Compute the partial of the RHS of the ODE w.r.t the ith control and
    % ith state
    [df_du(:,:,i),df_dx(:,:,i)] = planar_robots_ode_diff(X(:,i),U(:,i));
    
    % Compute partial of ith dynamic constraint to control and states
    %                  { -d( f(X(ti), U(ti), ti ) / dU(tj)  if i == j
    %   dc(i)/du(tk) = {
    %                  {                   0                if i ~= j
    %
    %                  { 2/(tf - t0) * D(i,k) * I_nxn - d( f(X(ti), U(ti), ti )/dX(tk)  if i == k
    %   dc(i)/dx(tk) = {
    %                  { 2/(tf - t0) * D(i,k) * I_nxn                                   if i ~= k
    dc_du = zeros(size(df_du));
    dc_dx = zeros(size(df_dx));
    for k = 1:N
        if (i == k)
            dc_du(:,:,k) = -df_du(:,:,i);
            dc_dx(:,:,k) = 2/(tf - t0) * D(i,k) * eye(nstates,nstates) - df_dx(:,:,i);
        else % i ~= k
            dc_dx(:,:,k) = 2/(tf - t0) * D(i,k) * eye(nstates,nstates);            
        end
    end
    
    % Compute the total partial of the ith dynamic constraint w.r.t to all
    % design variables
    offset = 1 + (i-1)*nstates;
    dc(offset:offset+nstates-1, :) = [dc_dtf, reshape(dc_du, nstates, dim*N, 1), reshape(dc_dx, nstates, nstates*N, 1)];
    
    % Compute partial of the final time/state constraint w.r.t. the control
    %   dcf/dU(ti) = -(tf - t0)/2 * w(i) * d( f(X(ti), U(ti), ti) )/dU(ti)
    dcf_du(:,:,i) = -(tf - t0)/2 * w(i) * df_du(:,:,i);
    
    % Compute partial of the final time/state constraint w.r.t. the state
    %   dcf/dX(ti) = -(tf - t0)/2 * w(i) * d( f(X(ti), U(ti), ti) )/dX(ti)
    dcf_dx(:,:,i) = -(tf - t0)/2 * w(i) * df_dx(:,:,i);    
end

% Compute sum_k ( w(k) * f(X(tk), U(tk), tk) )
Wk_mat = repmat(w', nstates, 1);
sum_k_WkFk = sum(Wk_mat .* fi, 2);

% Compute partial of the final time/state constraint w.r.t. the final time:
%   dcf/dtf = -0.5 * sum_k ( w(k) * f(X(tk), U(tk), tk) )
dcf_dtf = -0.5 * sum_k_WkFk;

% Compute the total partial of the final time constraint w.r.t to all
% design variables
dc(1 + N*nstates:end, :) = [dcf_dtf, reshape(dcf_du, nstates, dim*N, 1), reshape(dcf_dx, nstates, nstates*N, 1)];

% Lastly, convert dc to a sparse matrix
dc = sparse(dc);
end