function Prob = rdtAssign(x0, xf, x_lb, x_ub, u_lb, u_ub, iter, ...
						  name, old_tree, userdata, ...
                          odefun, ngen, neval, nsel, lp, OutputFcn)
% rdtAssign - Defines the problem structure for the RDT algorithm
%
%   Prob = rdtAssign(x0, xf, lb, ub, odefun, ngen, neval, nsel, lp) creates 
%   a structure containing all the necessary information for the RDT
%   implementation in Matlab. The inputs are described below:
%
%       x0      Known initial state of the system.
%       xf      Desired final state of the system.
%       lb      Lower bound on system states
%       ub      Upper bound on system states
%       odefun  Function handle which computes the dynamic constrains of 
%               system in question. The syntax of odefun must be:
%
%                   x_dot = odefun(x,u,t,Prob)
%
%       ngen    Function handle which randomly generates a system state.
%               Syntax for ngen must be:
%
%                   xr = ngen(Prob)
%
%       neval   Function handle which evaluates a system state and returns
%               a scalar value indicating its relative fitness.
%
%                   fitness = neval(x, Prob)
%
%       nsel    Function handle which typically returns the closest system
%               state to a given query state. This could be closest in the
%               euclidean sense or more complex for systems with holonomic
%               constraints. Inputs to this function are the query state,
%               xq, a matrix of nodes, V, in column format to test 
%               distances, and a row vector, Wv, of the associated node 
%               fitnesses. nsel must return the columnID from V of the
%               closest node.
%
%                   colID = nsel(xq, V, Wv)
%
%       lp      Function handle which computes a local plan between two
%               system states. It can return any number N intermediate
%               nodes in column format, Ni, and the associate edge weights 
%               between those intermediate nodes. The total path is a 
%               NS x (N+2) matrix [Ne, Ni, Nr] with edge weights a row
%               vector of length N+1. If no feasible path exists then Ni
%               must be empty.
%
%                   [Ni,We] = lp(Ne, Nr, Prob);
%
%   Prob = rdtAssign(..., iter) specifies how many expansion steps the RDT
%   algorithm should make. The default is iter = 30.
%
%   Prob = rdtAssign(..., iter, old_tree) instructs the RDT algorithm to
%   use an existing tree from a previous set of iterations.
%
%   Prob = rdtAssign(..., iter, old_tree, userdata) allows the user to
%   store any other data which will be ultimately passed to any of the
%   above function handles.

% if nargin < 16
%     OutputFcn = @rdtOutput;
%     if nargin < 15
%         userdata = [];
%         if nargin < 14
%             old_tree = [];
%             if nargin < 13
%                 name = 'Default';
%                 if nargin < 12
%                     iter=20;
%                     if nargin < 11
%                         error('At minimum, the first 7 arguments must be specified');
%                     end
%                 end
%             end
%         end
%     end
% end

Prob.name = name;
Prob.ns = length(x0);

if (isempty(x_lb)) Prob.x_lb = -inf*ones(size(x0));
else Prob.x_lb = x_lb; end

if (isempty(x_ub)) Prob.x_ub = inf*ones(size(x0));
else Prob.x_ub = x_ub; end

Prob.x_range = x_ub - x_lb;
Prob.iter = iter;
Prob.G = old_tree;
Prob.userdata = userdata;

Prob.u_lb = u_lb;
Prob.u_ub = u_ub;

% % Scale x0 and xf between -1 and 1
% Prob.x0 = (2*x0 - (ub + lb)) ./ Prob.x_range;
% if (isempty(xf)) Prob.xf = NaN*ones(size(x0));
% else
%     Prob.xf = (2*xf - (ub + lb)) ./ Prob.x_range;; 
% end

% Don't pre-scale states...
Prob.x0 = x0;
if (isempty(xf)) Prob.xf = NaN*ones(size(x0));
else
    Prob.xf = xf; 
end

% Function checker
isfhandle = @(fun)(isa(fun, 'function_handle'));

% odefun
if isfhandle(odefun)
    fname = func2str(odefun);
    if ( nargin(odefun) ~= 1 ) error([fname ' must accept 1 arguments.']); end
else
    error('Argument odefun must be a function handle.');
end
Prob.odefun = odefun;

% node_generator
if isfhandle(ngen)
    fname = func2str(ngen);
    if ( nargin(ngen) ~= 1 ) error([fname ' must accept 1 arguments.']); end
else
    error('Argument node_generator must be a function handle.');
end
Prob.node_generator = ngen;

% node_evaluate
if isfhandle(neval)
    fname = func2str(neval);
    if ( nargin(neval) ~= 2 ) error([fname ' must accept 2 arguments.']); end
else
    error('Argument node_evaluate must be a function handle.');
end
Prob.node_evaluate = neval;

% node_select
if isfhandle(nsel)
    fname = func2str(nsel);
    if ( nargin(nsel) ~= 4 ) error([fname ' must accept 3 arguments.']); end
else
    error('Argument node_select must be a function handle.');
end
Prob.node_select = nsel;

% local_planner
if isfhandle(lp)
    fname = func2str(lp);
    if ( nargin(lp) ~= 3 ) error([fname ' must accept 3 arguments.']); end
else
    error('Argument local_planner must be a function handle.');
end
Prob.local_planner = lp;

% Output function
if isfhandle(OutputFcn)
    fname = func2str(OutputFcn);
    if ( nargin(OutputFcn) ~= 6 ) error([fname ' must accept 6 arguments.']); end
else
    error('Argument OutputFcn must be a function handle.');
end
Prob.output_fcn = OutputFcn;

% Reset random generators
rand('state',sum(200*clock))
randn('state',sum(100*clock))
