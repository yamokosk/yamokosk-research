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
    bestPathScore = searchTree.bestPathScore;
    bestLeafID = searchTree.bestLeafID;
    bestRootID = searchTree.bestRootID;
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
        ti = Xi(end,m);
    
        % Based on the selected node, pick a average target to drive the search
        % towards
        if (ti >= target.tspan(2))
            disp(sprintf(iterformat, itercount, NaN, bestPathScore, bestRootID, bestLeafID, 'Selection failed.'));
            continue;
        end
        t_step = min(target.tspan(2), ti + timestep);
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
        [branchID,bestBranchEff] = evaluate(branch_candidates, target, neval);

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
    
searchTree.bestPathScore = bestPathScore;
searchTree.bestLeafID = bestLeafID;
searchTree.bestRootID = bestRootID;
exitflag = 1;



% =========================================================================
% initializeSearchTree(x0, targets, neval)
%
%   Performs some simple initialization steps for the planner, including:
%       * Initialize output function
%       * Initialize solution data structure, if needed
% =========================================================================
function G = initializeSearchTree(x0, target, neval)
G = graph();
[dof, numStartingNodes] = size(x0);
Tvar=diag(target.variance);
for n = 1:numStartingNodes
    Tbar = ppval(target.pp, x0(end));
    eff = nodeSensingEffectiveness(x0(:,n), Tbar, Tvar, neval);
    G = add_node(G, x0(:,n), eff, true);
end

% =========================================================================
% select(G, targets, minEff)
%
%   Probabilistically selects the next target to sense. Basic operation:
%       1. Probabilistically select the next node to expand based on the
%       node weights. Currently uses the stochastic universal sampling 
%       algorithm from the GA communitiy. Reference:
%            Baker, J.E., "Reducing bias and inefficiency in the selection 
%               algorithm", Proceedings of the Second International 
%               Conference on Genetic Algorithms and their application,
%               pp.14-21, 1987.
%       2. Then a target is selected which must meet the following
%       criteria:
%           I. Has a time greater than the selected node.
%               AND
%           IIa. Has a minimum effectiveness less than the required
%           minimum.
%               OR
%           IIb. Has the smallest effectiveness of all furutre targets.
% =========================================================================
function [Xsel, Wsel, IDsel] = select(G)

allNodes = G.V;
allNodeWeights = G.Vw;

% Probabilistically select the most fit node to expand
[Xsel,Wsel,IDsel] = selsus(allNodes,allNodeWeights,3);

% % Take best two or three
[junk, ind] = sort(Wsel,2,'descend');

Xsel = Xsel(:,ind(1:2));
Wsel = Wsel(ind(1:2));
IDsel = IDsel(ind(1:2));


% =========================================================================
% generate(Tj, N, ngen)
%
%   Generate a set of nodes for target Tj. Employs the user's  generation 
%   function to do all the heavy lifting.
% =========================================================================
function Vj = generate(t_step, Tbar, N, ngen)
Vj = [];
for n = 1:N
    for m = 1:N
        Vtemp = ngen(t_step, Tbar);
        if ( ~isempty(Vtemp) )
            Vj = [Vj, Vtemp];
            break;
        end
    end
end

% =========================================================================
% extend(Vj, Xi, lp)
%
%   Computes paths from the currently selected node, X, to the set of 
%   generated vantage points, Vj. Employs a user supplied local planning
%   function to determine if a feasible path exists.
% =========================================================================
function [path_candidates, path_weights] = extend(X, Vj, lp);
numVantagePts = size(Vj,2);
path_candidates = [];
path_weights = [];
c = 1;
for n = 1:numVantagePts
    % Determine if a feasable path exists.
    path = lp(X, Vj(:,n));
    
    if ( ~isempty(path.xi) )
        path_candidates(:,:,c) = path.xi;
        path_weights(c,:) = path.ew;
        c = c + 1;
    end
end

% =========================================================================
% evaluate(branches, targets, doCollisionCheck, ccheck)
%
%   Evaluates the branch candidates and returns the branch which has the
%   greatest mean sensing effectiveness and that is also collision free (if
%   doCollisionCheck is true). Note that the collision check is actually
%   delayed until after the mean sensing effectiveness scores are
%   calculated. This is done in hopes of reducing the number of collision
%   checks we need to actually perform.
% =========================================================================
function [branchID,bestBranchEff,bestBranchScore] = evaluate(branches, target, neval)

% Get the sizes of things and allocate some space
numCandidates = size(branches, 3);
numNodes = size(branches, 2);
%allBranchEff = zeros(numNodes, numTargets, numCandidates);
avgBranchEff = zeros(1,numCandidates);

% Compute the average sensing effectiveness for each branch. I am purposely
% delaying collision checking here since it is assumed that will be a
% computationally intensive task.
Tvar=diag(target.variance);
F = zeros(numCandidates,1);
f = zeros(numNodes,numCandidates);
for c = 1:numCandidates
    % Select only the intermediate nodes and the generate vantage point. We
    % have already computed the sensing effectiveness of the root node in a
    % prior iteration. No need to reinclude it here.
    branch = branches(:,1:end,c);
    
    % NEW NEW METHOD.. using unscented transform to estimate node
    % evaluation statistics
    t_branch = branch(end,:)';
    Tbar = ppval(target.pp, t_branch)';
    for n = 1:numNodes
        f(n,c) = nodeSensingEffectiveness(branch(:,n), Tbar(:,n), Tvar, neval);
    end
    F(c) = trapz(t_branch,f(:,c));
end

% Sort candidates by their average branch effectiveness
%[ignore,ind] = sort(avgBranchEff, 2, 'descend');
[ignore,ind] = sort(F, 1, 'descend');

branchID = ind(1);
bestBranchEff = f(:,ind(1))';
bestBranchScore = avgBranchEff(ind(1));


% =========================================================================
% solution_check(G, branchIDs, numTargets, minEff)
%
%   Checks the newly created branch to see if we have achieved the sensing
%   requirements. Again the requirement for success is simply whether we
%   have achieved the minimum sensing requirements for all the targets.
% =========================================================================
function [rootID, leafID, score] = solution_check(G, branchIDs, target, minEff)

leafID = branchIDs(end);

% Check to see if the recently added branch results in a successful path to
% the goal.
[candPathIDs, candDist, candEff] = get_shortest_path(G, leafID);
rootID = candPathIDs(1);
candPath = G.V(candPathIDs);

t = candPath(end,:);
F = trapz(t, candEff);
Fmin = (target.tspan(2) - target.tspan(1)) * minEff;
score = F/Fmin;


% =========================================================================
% add_branch(G, rootID, newBranch, branchEff)
%
%   Really a utility function to automate the addition of a collection of
%   nodes to the tree. The nodes are assumed to be in the order which they
%   should be connected.
% =========================================================================
function [G, new_ids] = add_branch(G, rootID, newBranch, branchWeights, branchEff)
numNewNodes = size(newBranch,2);

% Add the nodes to the graph
new_ids = zeros(1,numNewNodes);
for n = 1:numNewNodes
    [G,new_ids(n)] = add_node(G, newBranch(:,n), branchEff(n));
end

% Make all the new connections
G = add_edge(G, rootID, new_ids(1), branchWeights(1));
for n = 2:numNewNodes
    G = add_edge(G,new_ids(n-1),new_ids(n), branchWeights(n));
end

% Force a recomputation of all shortest paths since we have just added a
% new branch.
G = compute_shortest_paths(G);

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