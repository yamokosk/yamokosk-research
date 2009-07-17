function [searchTree,exitflag] = plannerSolve(x0, target, func_handles, options, varargin)
% plannerSolve - Solves the vantage point planning problem for my thesis!
%
%   Prob = plannerSolve(Prob) attempts to solve the vantage point planning
%   problem for a problem domain specified in Prob. See plannerAssign for 
%   information on constructing a proper Problem data structure.
%
%   DisplayLevel of display. 'off' displays no output; 'iter' displays output at each iteration; 'final' displays just the final output; 'notify' (default) displays output only if the function does not converge.
%   MaxIterMaximum number of iterations allowed
%   OutputFcnSpecify a user-defined function that the optimization function calls at each iteration.

defaultopt = struct('Display', 'notify', ...
                    'MaxIter', '100', ...
                    'MinSenEff', 0.85, ...
                    'SkewFactor', 0.5, ...
                    'TimeStep', 0, ...
                    'NumVantagePts', 4, ...
                    'NumExtensionAttempts', 3, ...
                    'DoCollisionCheck', false, ...
                    'PriorSearchTree', 'none', ...
                    'OutputFcn', []);

% If just 'defaults' passed in, return the default options in X
if nargin==1 && nargout <= 1 && isequal(x0,'defaults')
    searchTree = defaultopt;
    return
end

if nargin < 3,
    error('MATLAB:plannerSolve:NotEnoughInputs',...
        'PLANNERSOLVE requires at least two input arguments');
end

if nargin < 4, options = []; end

% Size some stuff up
%numberOfTargets = size(targets,2);
numberUserArguments = length(varargin);

% Get algorithm tuning parameters
printtype = optget(options,'Display',defaultopt);
maxiter = optget(options,'MaxIter',defaultopt);
mineff = optget(options,'MinSenEff',defaultopt);
skewfactor = optget(options,'SkewFactor',defaultopt);
timestep = optget(options,'TimeStep',defaultopt);
numberOfVantagePts = optget(options,'NumVantagePts',defaultopt);
numberExtensionAttempts = optget(options, 'NumExtensionAttempts', defaultopt);
doCollisionCheck = optget(options,'DoCollisionCheck',defaultopt);
searchTree = optget(options,'PriorSearchTree',defaultopt);

% Check and then create some function handles
checkFunction(func_handles.ngen, 2+numberUserArguments);
ngen = @(t, target) func_handles.ngen(t, target, varargin{:});

checkFunction(func_handles.lp, 2+numberUserArguments);
lp = @(x1, x2) func_handles.lp(x1, x2, varargin{:});

checkFunction(func_handles.neval, 2+numberUserArguments);
neval = @(x, target) func_handles.neval(x, target, varargin{:});

if (doCollisionCheck)
    checkFunction(func_handles.ngen, 1+numberUserArguments);
    ccheck = @(x) func_handles.collisionCheck(x, varargin{:});
else
    ccheck = 0;
end

if ischar(maxiter)
    if isequal(maxiter, '100')
        maxiter = 100;
    end
end

if (timestep == 0)
    timestep = (target.tspan(2) - target.tspan(1))/20;
end

if (ischar(searchTree))
    if ( isequal(searchTree, 'none') )
        searchTree = initializeSearchTree(x0, target, neval);
        bestPathScore = 0;
        bestLeafID = 0;
        bestRootID = 0;
    end
else
    searchTree = graph(searchTree);
    bestPathScore = searchTree.BestPathScore;
    bestLeafID = searchTree.BestLeafID;
    bestRootID = searchTree.BestRootID;
end

switch printtype
    case 'notify'
        prnt = 1;
    case {'none','off'}
        prnt = 0;
    case 'iter'
        prnt = 3;
    case 'final'
        prnt = 2;
    otherwise
        prnt = 1;
end

% Handle the output
outputfcn = optget(options,'OutputFcn',defaultopt);
if isempty(outputfcn)
    haveoutputfcn = false;
else
    haveoutputfcn = true;
end

header = ' Iteration       Score       Best score         Best path      Status';
iterformat = ' %5.0f%18.4f%15.4f       (%4d,%4d)      %s';

% Initialize the output function.
if haveoutputfcn
    stop = callOutputFcn(outputfcn, 'init', searchTree, best_path_length, best_path_score, -1, msg, varargin{:});
    if stop
        if  prnt > 0
            disp(output.message)
        end
        return;
    end
end

% Print out initial f(x) as 0th iteration
if prnt == 3
    disp(' ')
    disp(header)
    disp(sprintf(iterformat, 0, 0, bestPathScore, bestRootID, bestLeafID, 'Initialization'));
end

% Now just do the specified number of iterations
for itercount = 1:maxiter
    % Recalculate node weights
    searchTree = recalculate_node_weights(searchTree, mineff, skewfactor);
    
    % Probabilistically select a node and target pair.
    [Xi, Wi, i] = select(searchTree);
    numSelectedNodes = length(i);
    for m = 1:numSelectedNodes
        % Record that we are 'visiting' node i
        searchTree = record_visit(searchTree, i(m));
        
        % Get time of node we are trying to connect to
        ti = Xi(end,m);
    
        % Based on the selected node, pick a average target to drive the search
        % towards
        if (ti >= target.tspan(2))
            disp(sprintf(iterformat, itercount, NaN, bestPathScore, bestRootID, bestLeafID, 'Selection failed.'));
            continue;
		end
		% LAST THING TO TRY OUT - For each random time generate a random
		% state. Right now its random time and N random states for that
		% time.
		t_rand = (0.2 - 0.04)*rand(1) + 0.04;
		t_step = min(target.tspan(2), ti + t_rand);
%         t_step = min(target.tspan(2), ti + timestep);
		Tbar = ppval(target.pp, t_step);

        % Try a few times to extend the search tree from the selected node.
        branch_candidates = [];
        for extendcount = 1:numberExtensionAttempts
            % Generate a set of vantage points for target j
            Vj = generate(t_step, Tbar, numberOfVantagePts, ngen);
            if (isempty(Vj))
                break;
            end

            % Extend the tree to one of the generated nodes
            [branch_candidates, branch_weights] = extend(Xi(:,m), Vj, lp);
            if ( ~isempty(branch_candidates) )
                break;
            end
        end

         % Notify user that this iteration was a failure.
        if ( isempty( branch_candidates ) )
            disp(sprintf(iterformat, itercount, NaN, bestPathScore, bestRootID, bestLeafID, 'Extension failed.'));
            continue;
        end

        % Evaluate the branch candidates. If there is a valid branch amongst
        % the candidates, add the best one to the tree.
        [branchID,branchEff] = evaluate(branch_candidates, target, neval);

        % Its possible no feasible branch was found amongst the
        % candidates. Really only a possibility if we are doing collision
        % checking.
        if ( branchID < 0 )
            % Notify user we failed at this point.
            disp(sprintf(iterformat, itercount, NaN, bestPathScore, bestRootID, bestLeafID, 'Evaluation failed.'));
            continue;
        end

        % Add branch to search tree
        bestBranch = branch_candidates(:,2:end,branchID);
        bestBranchWeights = branch_weights(branchID,:);
        bestBranchEff = branchEff(:,2:end);
        [searchTree, branchIDs] = add_branch(searchTree, i(m), bestBranch, bestBranchWeights, bestBranchEff);
    
        % Check to see if this new branch gives us a valid solution.
        [rootID, leafID, iterationScore] = solution_check(searchTree, branchIDs, target, mineff);

        if ( iterationScore > bestPathScore )
            bestPathScore = iterationScore;
            bestLeafID = leafID;
            bestRootID = rootID;
        end
    end

    if prnt == 3
        disp(sprintf(iterformat, itercount, iterationScore, bestPathScore, bestRootID, bestLeafID, 'Search tree extended.'));
    end
end

% Notify output function we are done
if prnt > 1
    disp(' ')
    disp(['Search terminated.'])
end

% Recalculate node weights
searchTree = recalculate_node_weights(searchTree, mineff, skewfactor);
    
searchTree.BestPathScore = bestPathScore;
searchTree.BestLeafId = bestLeafID;
searchTree.BestRootId = bestRootID;
exitflag = 1;


% =========================================================================
% optget(options, fieldname, default)
%
%   A utility function to assist with the set/get of options.
% =========================================================================
function val = optget(options, fieldname, default)

if ( isfield(options, fieldname) )
    val = options.(fieldname);
else
    val = default.(fieldname);
end

% =========================================================================
% fcnchk(options, fieldname, default)
%
%   A utility function to assist with the set/get of options.
% =========================================================================
function checkFunction(fhandle, num_req_args)
% Function checker
isfhandle = @(fun)(isa(fun, 'function_handle'));

if isfhandle(fhandle)
    if ( nargin(fhandle) ~= num_req_args ) 
        error('PLANNER:plannerSolve:BadUserDefinedFunctionHandle',...
            ['User supplied ' func2str(fhandle) ' must accept ' num2str(num_req_args) ' arguments.']); 
    end
else
    error('PLANNER:plannerSolve:BadFunctionHandle',...
            ['User supplied ' func2str(fhandle) ' must accept is not a function handle.']); 
end


%--------------------------------------------------------------------------
function [xOutputfcn, optimValues, stop] = callOutputFcn(outputfcn,x,xOutputfcn,state,iter,...
    numf,how,f,varargin)
% CALLOUTPUTFCN assigns values to the struct OptimValues and then calls the
% outputfcn.
%
% state - can have the values 'init','iter', or 'done'.
% We do not handle the case 'interrupt' because we do not want to update
% xOutputfcn or optimValues (since the values could be inconsistent) before calling
% the outputfcn; in that case the outputfcn is called directly rather than
% calling it inside callOutputFcn.

% For the 'done' state we do not check the value of 'stop' because the
% optimization is already done.
optimValues.iteration = iter;
optimValues.fval = f;
optimValues.procedure = how;

% callOutputFcn is not called with state='interrupt', that's why
% this value is missing in the switch-case below. When state='interrupt',
% the output function is called directly, not via callOutputFcn.
switch state
    case {'iter','init'}
        stop = outputfcn(searchTree,optimValues,state,varargin{:});
    case 'done'
        stop = false;
        outputfcn(xOutputfcn,optimValues,state,varargin{:});
    otherwise
        error('MATLAB:fminsearch:InvalidState', ...
            'Unknown state in CALLOUTPUTFCN.')
end