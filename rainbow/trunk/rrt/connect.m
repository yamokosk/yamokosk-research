function Result = connect(t0, x0, xf, odefun)
nstates = length(x0);
dim = nstates/2;

% Create Gauss collocation points
N = 30;
[D, Dbar, nodes, w] = legdiff_G(N);

% Setup optimization problem
z0 = [1; zeros(dim*N,1); zeros(nstates*N,1)];
lb = [(N+1)*2e-3; -5*ones(dim*N,1); -inf*ones(nstates*N,1)];
ub = [inf; 5*ones(dim*N,1); inf*ones(nstates*N,1)]; 

% % Use our guess point to figure out the constraint pattern
m = nstates*(N+1);
n = length(z0);

% % tf pattern
% dcdtf = ones(nstates*(N+1),1);
% 
% % U's pattern
% dcdu = [];
% for ci = 1:N
%     dcdu = blkdiag(dcdu,ones(nstates, dim));
% end
% [row, col] = size(dcdu);
% dcdu = [dcdu; ones(nstates,col)];
% 
% % X's pattern
% dcdx = ones(nstates*(N+1), nstates*N);
% 
% % Put it all together
% ConsPattern = [dcdtf, dcdu, dcdx];
ConsPattern = sparse(m,n);

% Define TOMLAB problem
Prob = conAssign(@costfun, @costgrad, @costHess, sparse(1,1), lb, ub, 'Robot2D', z0, ...
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
Prob.NumDiff = 6; % Tell TOMLAB that snopt is estimating
Prob.ConsDiff = 6; % all derivatives (gradient and Jacobian)
Prob.ConsPattern = confun_diff(rand(n,1),Prob);

% Prints maximum amount of information for debugging purposes
Prob.SOL.PrintFile = 'snoptp.out'; % New name for Print File
Prob.SOL.SummFile = 'snopts.out'; % New name for Summary File
Prob.PriLevOpt = 3;
Prob.SOL.optPar(1) = 111111; % Major print level, combination of six 0/1
Prob.SOL.optPar(2) = 10; % Minor print level, 0, 1 or 10. 10 is maximum
Prob.SOL.optPar(5) = 1; % Print Frequency
Prob.SOL.optPar(6) = 1; % Summary Frequency
Prob.SOL.optPar(7) = 1; % Solution yes. 0 = Solution not printed
Prob.SOL.optPar(8) = 1; % Full options listing, not default

% Uncomment the following to make SNOPT totally silent (no debug/output
% information)
% Prob.SOL.PrintFile = '';
% Prob.SOL.SummFile = '';
% Prob.SOL.optPar(2) = 0;
% Prob.SOL.optPar(3) = 0;

% Do optimization
%Prob.optParam = optParamDef('snopt',Prob.probType, 1, m, m);
Result = tomRun('snopt', Prob);
%[zopt, f_k] = fmincon(@costfun, z0, [], [], [], [], lb, ub, @confun, opts, Prob);

% Get answer suitable for output
% [tf_opt,U_opt,X_opt] = unpack(zopt,dim,N);
% Xf = x0;
% for n = 1:N
%     Xf = Xf + w_hat(n) * odefun(X_opt(:,n),U_opt(:,n));
% end

end







