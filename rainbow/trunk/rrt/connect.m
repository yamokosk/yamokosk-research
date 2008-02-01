function [U_opt,X_opt,Xf,CPUTime] = connect_min_effort(t0, X0, tf, Xf, N, odefun)

% Create N Gauss collocation points
[D, Dbar, nodes, w] = legdiff_G(N);

% Setup optimization problem
ns = length(x0);
nc = nstates/2;
y0 = [zeros(dim*N,1); zeros(nstates*N,1)];
lb = [(N+1)*2e-3; -5*ones(dim*N,1); -inf*ones(nstates*N,1)];
ub = [inf; 5*ones(dim*N,1); inf*ones(nstates*N,1)]; 

% % Use our guess point to figure out the constraint pattern
m = nstates*(N+1);
n = length(z0);
ConsPattern = sparse(m,n);

% Define TOMLAB problem
Prob = conAssign(@cost_effort, @cost_effort_grad, @cost_effort_hess, sparse(1,1), lb, ub, 'Robot2D', z0, ...
                            [], lb(1), ...
                            [], [], [], @confun, @confun_diff, [], ConsPattern, zeros(m,1), zeros(m,1));

% User data to pass to callback functions
Prob.user.t0 = t0;
Prob.user.x0 = x0;
Prob.user.xf = xf;
Prob.user.D = D;
Prob.user.Dbar = Dbar;
Prob.user.nodes = nodes;
Prob.user.w = w;
Prob.user.dim = dim;
Prob.user.odefun = odefun;

% Performance tweaks
% Prob.SOL.optPar(40) = 0; % Use Nonderivative instead of Derivative Linesearch
Prob.NumDiff = 0;%6; % Tell TOMLAB that snopt is estimating
Prob.ConsDiff = 0;%6; % all derivatives (gradient and Jacobian)
zr = rand(n,1);
Prob.ConsPattern = confun_diff(zr,Prob);
%Prob.SOL.optPar(13) = 1;

% Prints maximum amount of information for debugging purposes
% Prob.SOL.PrintFile = 'snoptp.out'; % New name for Print File
% Prob.SOL.SummFile = 'snopts.out'; % New name for Summary File
% Prob.PriLevOpt = 3;
% Prob.SOL.optPar(1) = 111111; % Major print level, combination of six 0/1
% Prob.SOL.optPar(2) = 10; % Minor print level, 0, 1 or 10. 10 is maximum
% Prob.SOL.optPar(5) = 1; % Print Frequency
% Prob.SOL.optPar(6) = 1; % Summary Frequency
% Prob.SOL.optPar(7) = 1; % Solution yes. 0 = Solution not printed
% Prob.SOL.optPar(8) = 1; % Full options listing, not default

% Uncomment the following to make SNOPT totally silent (no debug/output
% information)
Prob.SOL.PrintFile = '';
Prob.SOL.SummFile = '';
Prob.SOL.optPar(2) = 0;
Prob.SOL.optPar(3) = 0;

% Do optimization
%Prob.optParam = optParamDef('snopt',Prob.probType, 1, m, m);
Result = tomRun('snopt', Prob)
%[zopt, f_k] = fmincon(@costfun, z0, [], [], [], [], lb, ub, @confun, opts, Prob);

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







