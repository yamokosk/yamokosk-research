function [searchTree, exitflag, bestScore, bestLeafID] = plannerSolve(x0, target, func_handles, options, varargin)
% plannerSolve - Solves the vantage point planning problem for my thesis!
%
%   Prob = plannerSolve(Prob) attempts to solve the vantage point planning
%   problem for a problem domain specified in Prob. See plannerAssign for 
%   information on constructing a proper Problem data structure.
%
%   DisplayLevel of display. 'off' displays no output; 'iter' displays output at each iteration; 'final' displays just the final output; 'notify' (default) displays output only if the function does not converge.
%   MaxIterMaximum number of iterations allowed
%   OutputFcnSpecify a user-defined function that the optimization function calls at each iteration.

defaultopt = struct('Display', 'iter', ...
                    'NumIterations', '100', ...
                    'NumStepsPerIteration', '3', ...
                    'NumNodesToSelectPerStep', '3', ...
                    'MinSenEff', 0.85, ...
                    'SkewFactor', 0.5, ...
                    'MinTimeStep', 1e-3, ...
                    'MaxTimeStep', 1e-1, ...
                    'NumVantagePts', 4, ...
                    'NumExtensionAttempts', 3, ...
                    'DoCollisionCheck', false, ...
                    'PriorSearchTree', 'none', ...
                    'OutputFcn', []);

% If just 'defaults' passed in, return the default options in X
if nargin==1 && nargout <= 1 && isequal(x0,'defaults')
    searchTree = defaultopt;
    return;
end

if nargin < 3,
    error('MATLAB:plannerSolve:NotEnoughInputs',...
        'PLANNERSOLVE requires at least two input arguments');
end

if nargin < 4, options = []; end

numberUserArguments = length(varargin);

% Get all the algorithm options
[prnt,numiterations,numStepsPerIteration,mineff,skewfactor,minstep,maxstep,numberOfVantagePts,numNodesToSelect ...
 numberExtensionAttempts,doCollisionCheck,searchTree] = set_algorithm_options(options, defaultopt);

% Check and then create some function handles
checkFunction(func_handles.ngen, 2+numberUserArguments);
ngen = @(t, target) func_handles.ngen(t, target, varargin{:});

checkFunction(func_handles.lp, 2+numberUserArguments);
lp = @(x1, x2) func_handles.lp(x1, x2, varargin{:});

checkFunction(func_handles.neval, 2+numberUserArguments);
neval = @(x, target) func_handles.neval(x, target, varargin{:});

if (doCollisionCheck)
    checkFunction(func_handles.collisionCheck, 1+numberUserArguments);
    ccheck = @(x) func_handles.collisionCheck(x, varargin{:});
else
    ccheck = 0;
end

outputfcn = optget(options,'OutputFcn',defaultopt);
if isempty(outputfcn)
    haveoutputfcn = false;
else
    haveoutputfcn = true;
end

% Create or recover initial search tree
if (ischar(searchTree))
    if ( isequal(searchTree, 'none') )
        searchTree = initializeSearchTree(x0, target, neval);
    end
else
    searchTree = graph(searchTree);
    if ( searchTree.UserSenEff < mineff )
        searchTree.BestPathDist = inf;
        fprintf(1,'BestPathDist getting reset.\n');
    end
    searchTree.UserSenEff = mineff;
end

% Some stuff for printing
headerFormat = '%-10s%-10s%-20s\n';
iterFormat = '%-10d%-10.2f%-20s\n';

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
    fprintf(1,'\n');
	fprintf(1,headerFormat,'Iter','Score','Status');
    fprintf(1,iterFormat,0,0,'Initialized.');
end

% Now just do the specified number of iterations
bestScore = -inf;
bestLeafID = -1;
for itercount = 1:numiterations
        
    % Prepare to do a step. Going to record the max node score after each
    % step and whether any step finds a solution.
    iterScores = zeros(numStepsPerIteration,1);
    iterSolutionFound = zeros(numStepsPerIteration,1);
    
    % Do the step loop
    for stepcount = 1:numStepsPerIteration
        % Recalculate node weights at the start of each step
        searchTree = recalculate_node_weights(searchTree, mineff, skewfactor);
    
        % Probabilistically select a node and target pair.
        [Xi, Wi, i] = select(searchTree,numNodesToSelect);

        % Randomly generate times in the future for which we will generate
        % vantage points
        ti = Xi(end,:); % Time at currently selected nodes
        t_rand = (maxstep - minstep)*rand(1,numNodesToSelect) + minstep*ones(1,numNodesToSelect);
        tf = target.tspan(2)*ones(1,numNodesToSelect);
        t_step = min( [tf; ti + t_rand] );
        Tbar = ppval(target.pp, t_step )';

        % Prepare to expand a few nodes.
        stepScores = zeros(numNodesToSelect,1);
        stepSolutionFound = zeros(numNodesToSelect,1);
    
        % For each selected node, generate a vantage point and try to connect
        % to it.
        for m = 1:numNodesToSelect
            % Record that we are 'visiting' node i
            searchTree = record_visit(searchTree, i(m));

            % Try a few times to extend the search tree from the selected node.
            branch_candidates = [];
            for extendcount = 1:numberExtensionAttempts
                % Generate a set of vantage points for target j
                Vj = generate(t_step(m), Tbar(:,m), numberOfVantagePts, ngen);
                if (isempty(Vj))
                    break;
                end

                % Extend the tree to one of the generated nodes
                [branch_candidates, branch_weights] = extend(Xi(:,m), Vj, lp);
                if ( ~isempty(branch_candidates) )
                    break;
                end
            end

            if ( isempty(branch_candidates) )
                continue;
            end

            % Evaluate the branch candidates. If there is a valid branch amongst
            % the candidates, add the best one to the tree.
            [branchID,branchEff] = evaluate(branch_candidates, target, neval, doCollisionCheck, ccheck);

            if ( isempty(branchID) )
                continue;
            end

            % Add branch to search tree
            bestBranch = branch_candidates(:,2:end,branchID);
            bestBranchWeights = branch_weights(branchID,:);
            bestBranchEff = branchEff(:,2:end);
            [searchTree, branchIDs] = add_branch(searchTree, i(m), bestBranch, bestBranchWeights, bestBranchEff);

            % Check to see if this new branch gives us a valid solution.
            [stepScores(m,1), stepSolutionFound(m,1), searchTree] = solution_check(searchTree, branchIDs, target, mineff);
            
            if ( stepScores(m,1) > bestScore )
                bestScore = stepScores(m,1);
                bestLeafID = branchIDs(end);
            end
        end
        
        iterScores(stepcount,1) = max(stepScores);
        iterSolutionFound(stepcount,1) = ~isempty(find(stepSolutionFound > 0,1));
    end % END OF STEP LOOP
    
    if prnt == 3
        fprintf(1,iterFormat,itercount,max(iterScores),'Iteration complete.');
    end
end % END OF ITERATION LOOP

% Notify output function we are done
if prnt > 1
    disp(' ')
    disp(['Search terminated.'])
end

% Recalculate node weights
searchTree = recalculate_node_weights(searchTree, mineff, skewfactor);
exitflag = 1;


% =========================================================================
% set_algorithm_options(x0, target, func_handles, options, defaultopt, numberUserArguments)
%
%   A utility function to assist with the set/get of options.
% =========================================================================
function [prnt,maxiter,numStepsPerIteration,mineff,skewfactor,minstep,maxstep,numberOfVantagePts,numNodesToSelect ...
          numberExtensionAttempts,doCollisionCheck,searchTree] = ...
          set_algorithm_options(options, defaultopt)

% Get algorithm tuning parameters
printtype = optget(options,'Display',defaultopt);
maxiter = optget(options,'NumIterations',defaultopt);
numStepsPerIteration = optget(options,'NumStepsPerIteration',defaultopt);
numNodesToSelect = optget(options,'NumNodesToSelectPerStep',defaultopt);
mineff = optget(options,'MinSenEff',defaultopt);
skewfactor = optget(options,'SkewFactor',defaultopt);
minstep = optget(options,'MinTimeStep',defaultopt);
maxstep = optget(options,'MaxTimeStep',defaultopt);
numberOfVantagePts = optget(options,'NumVantagePts',defaultopt);
numberExtensionAttempts = optget(options, 'NumExtensionAttempts', defaultopt);
doCollisionCheck = optget(options,'DoCollisionCheck',defaultopt);
searchTree = optget(options,'PriorSearchTree',defaultopt);

if ischar(maxiter)
    if isequal(maxiter, '100')
        maxiter = 100;
    end
end

if ischar(numStepsPerIteration)
    if isequal(numStepsPerIteration, '3')
        numStepsPerIteration = 3;
    end
end

if ischar(numNodesToSelect)
    if isequal(numNodesToSelect, '3')
        numNodesToSelect = 3;
    end
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