function dc = con_min_effort_grad(y,Prob)
% Unpack data
t0 = Prob.user.t0;
tf = Prob.user.tf;
nu = Prob.user.nu;
ns = Prob.user.ns;
N = Prob.user.N;
D = Prob.user.D;
w = Prob.user.w;

U = reshape(y(1:nu*N),nu,N);
X = reshape(y(nu*N + 1:end), ns, N);

% Allocate space for constraints
fi = zeros(ns,N);
df_du = zeros(ns, nu, N);
df_dx = zeros(ns, ns, N);

dc = zeros(ns * (N+1), length(y));
dcf_du = zeros(ns, nu, N);
dcf_dx = zeros(ns, ns, N);

% Compute constraints and its partial derivatives
for i = 1:N
    % Compute RHS of ODE at Xi, Ui, ti
    fi(:,i) = Prob.user.odefun(X(:,i),U(:,i));

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
            dc_dx(:,:,k) = 2/(tf - t0) * D(i,k) * eye(ns,ns) - df_dx(:,:,i);
        else % i ~= k
            dc_dx(:,:,k) = 2/(tf - t0) * D(i,k) * eye(ns,ns);            
        end
    end
    
    % Compute the total partial of the ith dynamic constraint w.r.t to all
    % design variables
    offset = 1 + (i-1)*ns;
    dc(offset:offset+ns-1, :) = [reshape(dc_du, ns, nu*N, 1), reshape(dc_dx, ns, ns*N, 1)];
    
    % Compute partial of the final time/state constraint w.r.t. the control
    %   dcf/dU(ti) = -(tf - t0)/2 * w(i) * d( f(X(ti), U(ti), ti) )/dU(ti)
    dcf_du(:,:,i) = -(tf - t0)/2 * w(i) * df_du(:,:,i);
    
    % Compute partial of the final time/state constraint w.r.t. the state
    %   dcf/dX(ti) = -(tf - t0)/2 * w(i) * d( f(X(ti), U(ti), ti) )/dX(ti)
    dcf_dx(:,:,i) = -(tf - t0)/2 * w(i) * df_dx(:,:,i);    
end

% Compute the total partial of the final time constraint w.r.t to all
% design variables
dc(1 + N*ns:end, :) = [reshape(dcf_du, ns, nu*N, 1), reshape(dcf_dx, ns, ns*N, 1)];

% Lastly, convert dc to a sparse matrix
dc = sparse(dc);
end