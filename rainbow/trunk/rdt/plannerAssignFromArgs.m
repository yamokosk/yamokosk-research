function Prob = plannerAssignFromArgs(x0, x_lb, x_ub, u_lb, u_ub, iter, f, varargin)
% plannerAssign - Defines the problem structure for the vantage point
%   planning problem.
%
%   Prob = plannerAssign(K, x0, x_lb, x_ub, u_lb, u_ub, iter, fhandles) 
%   creates a structure containing all the required information for the 
%   vantage point planning algorithm in Matlab. The inputs are described 
%   below:
%
%       targets Sensing targets. 
%       x0      Desired initial states of the system or dimension of the
%               system. (Scalar or NxM initial conditions)
%       x_lb    Lower bound on system states. (Nx1 vector)
%       x_ub    Upper bound on system states. (Nx1 vector)
%       u_lb    Lower bound on system's control inputs. (Px1 vector)
%       u_ub    Upper bound on system's control inputs. (Px1 vector)
%       iter    Specifies how many expansion steps the planner should make. 
%               The default is 30.
%       f       Structure of function handles:
%       f.ngen      Function handle to generate vantage points for 
%                   target. Syntax for ngen must be:
%
%                       xr = ngen(target, udata)
%
%       f.neval     Function handle which computes the sensing performance
%                   of a system state for a given set of targets. It should
%                   return a scalar value between 0 and 1.
%
%                       fitness = neval(state, targets, udata)
%
%       f.nsel      Function handle which typically returns the closest 
%                   system state to a given query state. This could be 
%                   closest in the euclidean sense or more complex for 
%                   systems with holonomic constraints. Inputs to this 
%                   function are the query state, xq, a matrix of nodes, V, 
%                   in column format to test distances, and a row vector, 
%                   Wv, of the associated node fitnesses. nsel must return 
%                   the columnID from V of the closest node.
%
%                       colID = nsel(xq, V, Wv)
%
%       f.lp        Function handle which computes a local plan between two
%                   system states. It can return any number k intermediate
%                   nodes in column format, Ni, and the associate edge 
%                   weights between those intermediate nodes. The total 
%                   path is a Nx(k+2) matrix [Ne, Ni, Nr] with edge 
%                   weights a row vector of length N+1. If no feasible path 
%                   exists then Ni must be empty.
%
%                       xi = lp(x0, xf, udata);
%
%       f.goalfun   Function handle which informs the planner whether the 
%                   all goal criteria have been statisfied.
%
%                       boolean = goalfun(Prob);
%
%   plannerAssign(..., name) Will give the problem/solution a user defined
%   name. It is used in archiving and logging the results.
%
%   plannerAssign(..., name, udata) allows one to pass user-defined data to
%   any of the user-specified function handles.
%
%   plannerAssign(..., name, udata, outfun) specifies an output function
%   handle. See documentation on the ODE output function handle for more
%   information.

min_num_args = 7;

% Check that we have at least the required minimum inputs
if (nargin < min_num_args)
    error([num2str(min_num_args) ' arguments is a minumum requirement']);
end
    
% Generate defaults for the variable inputs
name = ['default_' datestr(now, 30)];
udata = [];
f.outfun = @plannerOutput;

% Get variable inputs if they exist
if (nargin > min_num_args) name = [varargin{1}];end
if (nargin > min_num_args+1) udata = varargin{2}; end
if (nargin > min_num_args+2) f.outfun = varargin{3};end

% Now check we have valid values for the required inputs.
% Initial condition can be any of the following:
%   1. Scalar indicating number of states of the system
%   2. Column vector or matrix of initial conditions of the system.
x0_int = []; N = 1;
if ( isScalar(x0) )
    N = x0;
    x0_int = zeros(N, 1);
else
    x0_int = x0;
    N = size(x0_int,1);
end

% State lower and upper bounds can be either be empty or column vectors
x_lb_int = x_lb;
if (isempty(x_lb)) x_lb_int = -inf*ones(N,1); end
if ( size(x_lb_int,1) ~= N )
    error('State lower bound must be same dimension as system');
end

x_ub_int = x_ub;
if (isempty(x_ub)) x_ub_int = inf*ones(N,1); end
if ( size(x_ub_int,1) ~= N )
    error('State upper bound must be same dimension as system');
end

% Control lower and upper bounds must be column vectors.
u_lb_int = u_lb;
if (isempty(u_lb_int)) 
    error('Control lower bound was empty.');
end

u_ub_int = u_ub;
if (isempty(u_ub_int)) 
    error('Control upper bound was empty.');
end

% Iterations
if (isempty(iter)) iter = 30; end

% odefun
%checkFunction(f, 'odefun', 4);

% node_generate
checkFunction(f, 'ngen', 2);

% node_evaluate
checkFunction(f, 'neval', 3);

% node_select
checkFunction(f, 'nsel', 3);

% local_planner
checkFunction(f, 'lp', 3);

% goalfun
%checkFunction(f, 'goalfun', 1);

% Output function
checkFunction(f, 'outfun', 3);

% Create problem structure
Prob = struct('name',               name, ...
              'system_dimension',   N, ...
              'iterations',         iter, ...
              'x0',                 x0_int, ...
              'x_range',            x_ub_int - x_lb_int, ...
              'x_lb',               x_lb_int, ...
              'x_ub',               x_ub_int, ...
              'u_range',            u_ub_int - u_lb_int, ...
              'u_lb',               u_lb_int, ...
              'u_ub',               u_ub_int, ...
              'func_handles',       f, ...
              'solution',           [], ...
              'userdata',           udata);
                        
% Reset random generators
rand('state',sum(200*clock))
randn('state',sum(100*clock))
end

function ret = isScalar(value)
ret = true;
if (max(size(value)) > 1)
    ret = false;
end
end

function checkFunction(f, fname, num_req_args)
% Function checker
isfhandle = @(fun)(isa(fun, 'function_handle'));

if ( isfield(f,fname))
    if isfhandle(f.(fname))
        fname_user = func2str(f.(fname));
        if ( nargin(f.(fname)) ~= num_req_args ) error([fname_user ' must accept ' num2str(num_req_args) ' arguments.']); end
    end
else
    error([fname ' function handle was not specified.']);
end
end 
