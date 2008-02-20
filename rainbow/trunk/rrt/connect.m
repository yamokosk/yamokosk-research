function [U_opt,X_opt,Xf,CPUTime] = connect_min_effort(t0, x0, tf, xf, N, odefun)

% Create N Gauss collocation points
[D, Dbar, tau, w] = legdiff_G(N);

% Setup optimization problem
ns = length(x0);
nu = nstates/2;
y0 = [zeros(dim*N,1); zeros(nstates*N,1)];                        % Initial guess
y_lb = [(N+1)*2e-3; -5*ones(dim*N,1); -inf*ones(nstates*N,1)];    % Lower bounds on y
y_ub = [inf; 5*ones(dim*N,1); inf*ones(nstates*N,1)];             % Upper bounds on y

nc = length(y0);    
c_lb = zeros(nc,1);     % Lower bound on constraints
c_ub = zeros(nc,1);     % Upper bound on constraints

% Define TOMLAB problem
Prob = conAssign(@cost_min_effort, @cost_min_effort_grad, @cost_min_effort_hess, [], y_lb, y_ub, 'Robot2D', y0, ...
                 [], lb(1), ...
                 [], [], [], @con_min_effort, @con_min_effort_grad, [], [], c_lb, c_ub);

% User data to pass to callback functions
Prob.user = struct('tau', tau, 't0', t0, 'x0', x0, 'tf', tf, 'xf', xf, ...
                   'D', D, 'Dbar', Dbar, 'N', N, 'w', w, 'ns', ns, 'nu', nu, ...
                   'odefun', odefun);

% Performance tweaks
Prob.NumDiff = 0;   % Tell TOMLAB that MATLAB is estimating
Prob.ConsDiff = 0;  % all derivatives (via callback functions)

% Compute the sparsity pattern for the gradient of the constraints function
zr = rand(n,1);
Prob.ConsPattern = confun_diff(zr,Prob);

% Makes SNOPT totally silent (no debug/output information)
Prob.SOL.PrintFile = '';
Prob.SOL.SummFile = '';
Prob.SOL.optPar(2) = 0;
Prob.SOL.optPar(3) = 0;

% Do optimization
Result = tomRun('snopt', Prob)

% Get answer suitable for output
[tf_opt,U_opt,X_opt] = unpack(Result.x_k,dim,N);

% Compute sum_k ( w(k) * f(X(tk), U(tk), tk) )
fi = zeros(nstates,N);
for i = 1:N
    fi(:,i) = odefun(X_opt(:,i),U_opt(:,i));
end
Wk_mat = repmat(w', nstates, 1);
sum_k_WkFk = sum(Wk_mat .* fi, 2);

Xf = x0 + (tf_opt - t0)/2 * sum_k_WkFk;
CPUTime = Result.CPUtime;
end







