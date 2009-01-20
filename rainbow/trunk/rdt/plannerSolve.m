function [searchTree,exitflag] = plannerSolve(x0, targets, func_handles, options, varargin)
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
                    'MaxIter', '2*numberOfTargets', ...
                    'MinSenEff', 0.85, ...
                    'SkewFactor', 0, ...
                    'NumVantagePts', 4, ...
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
numberOfTargets = size(targets,2);
numberUserArguments = length(varargin);

% Get algorithm tuning parameters
printtype = optget(options,'Display',defaultopt);
maxiter = optget(options,'MaxIter',defaultopt);
mineff = optget(options,'MinSenEff',defaultopt);
skewfactor = optget(options,'SkewFactor',defaultopt) * numberOfTargets;
numberOfVantagePts = optget(options,'NumVantagePts',defaultopt);
doCollisionCheck = optget(options,'DoCollisionCheck',defaultopt);
searchTree = optget(options,'PriorSearchTree',defaultopt);

% Check and then create some function handles
checkFunction(func_handles.ngen, 1+numberUserArguments);
ngen = @(target) func_handles.ngen(target, varargin{:});

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
    if isequal(maxiter, '2*numberOfTargets')
        maxiter = 10000*numberOfTargets;
    end
end

if (ischar(searchTree))
    if ( isequal(searchTree, 'none') )
        searchTree = initializeSearchTree(x0, targets, neval);
        best_path_length = inf;
        best_path_score = 0;
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

% Handle the output
outputfcn = optget(options,'OutputFcn',defaultopt);
if isempty(outputfcn)
    haveoutputfcn = false;
else
    haveoutputfcn = true;
end

header = ' Iteration       min L(G)       max S(G)         Result';
iterformat = ' %5.0f%18.4f%15.4g          %s';

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
    disp(sprintf(iterformat, 0, best_path_length, best_path_score, 'Initialization'));
end

% Do some initialization stuff
% outfun('init', '', Prob);
% sol = struct();
% if isempty(Prob.solution)
%     sol = initialize(Prob, targets);
% else
%     sol = Prob.solution;
% end    

% Create local solution variables
% best_path_length = sol.bestPathLength;
% G = sol.connectivityGraph;

% Temporary storage for solutions found during this call.
goodPathEndpoints = zeros(50,1);
goodPathDistances = zeros(50,1);
goodPathScores = zeros(50,1);
goodPathIndex = 1;

% Now just do the specified number of iterations
for itercount = 1:maxiter
    % Recalculate node weights
    searchTree = recalculate_node_weights(searchTree, mineff, skewfactor);
    
    % Probabilistically select a node and target pair.
    [Xsel, Wsel, IDsel, Tj] = select(searchTree, targets, mineff);
    
    % Generate a set of vantage points for target j
    Vj = generate(Tj, numberOfVantagePts, ngen);
    
    % Extend the tree to one of the generated nodes
    [branch_candidates, branch_weights] = extend(Xsel, Vj, lp);
    if ( isempty(branch_candidates) )
        % Notify user that this iteration was a failure.
        disp(sprintf(iterformat, itercount, best_path_length, best_path_score, 'Extension failed.'));
        continue;
    end

    % Evaluate the branch candidates. If there is a valid branch amongst
    % the candidates, add the best one to the tree.
    [bestBranch, bestBranchWeights, bestBranchEff] = evaluate(branch_candidates, branch_weights, targets, neval, doCollisionCheck, ccheck);

    % Its possible no feasible branch was found amongst the
    % candidates. Really only a possibility if we are doing collision
    % checking.
    if ( isempty(bestBranch) )
        % Notify user we failed at this point.
        disp(sprintf(iterformat, itercount, best_path_length, best_path_score, 'Evaluation failed.'));
        continue;
    end
    [searchTree, branchIDs] = add_branch(searchTree, IDsel, bestBranch, bestBranchWeights, bestBranchEff);

    % Check to see if this new branch gives us a valid solution.
    [path, pathEff, pathDist] = solution_check(searchTree, branchIDs, numberOfTargets, mineff);
    msg = '';
    if ( ~isempty(path) )
        if ( pathDist < best_path_length )
            best_path_length = pathDist;
        end

        pathScore = mean(pathEff);
        if ( pathScore > best_path_score )
            best_path_score = pathScore;
        end

        goodPathEndpoints(goodPathIndex) = path(end);
        goodPathDistances(goodPathIndex) = pathDist;
        goodPathScores(goodPathIndex) = pathScore;
        goodPathIndex = goodPathIndex + 1;

        msg = 'New solution found!';
    else
        msg = 'Search tree extended.';
    end

    if prnt == 3
        disp(sprintf(iterformat, itercount, best_path_length, best_path_score, msg));
    end
%         if (haveoutputfcn)
%             stop = callOutputFcn(outputfcn, 'iter', searchTree, best_path_length, best_path_score, itercount, msg, varargin{:});
%             if stop
%                 if  prnt > 0
%                     disp(output.message)
%                 end
%                 return;
%             end
%         end        
end

% Notify output function we are done
if prnt > 1
    disp(' ')
    disp(['Search terminated.'])
end
exitflag = 1;

% Copy things back into solution structure
% sol.bestPathLength = best_path_length;
% sol.connectivityGraph = searchTree;
% sol.completeNodes = [sol.completeNodes; goodPathEndpoints(1:goodPathIndex-1)];
% sol.completeLengths = [sol.completeLengths; goodPathDistances(1:goodPathIndex-1)];
% sol.completeScores = [sol.completeScores; goodPathScores(1:goodPathIndex-1)];
% 
% Prob.solution = sol;


% =========================================================================
% initializeSearchTree(x0, targets, neval)
%
%   Performs some simple initialization steps for the planner, including:
%       * Initialize output function
%       * Initialize solution data structure, if needed
% =========================================================================
function G = initializeSearchTree(x0, targets, neval)
G = graph();
[dof, numStartingNodes] = size(x0);
for n = 1:numStartingNodes
    eff = nodeSensingEffectiveness(x0(:,n), targets, neval);
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
function [Xcurr, Wcurr, IDcurr, Tj] = select(G, targets, minEff)
% Potential speedup here. Targets is a static data stucture. So constantly
% recalulating the max target time is kinda stupid.
maxTargetTime = max(targets(end,:));
allNodes = G.V;
allNodeWeights = G.Vw;

% Generate list of all possible nodes we could expand. Criteria is all
% nodes with a time less than the most future target.
origNodeID = find( allNodes(end, :) < maxTargetTime );
if ( isempty(origNodeID) )
    % Should not normally happen.. so throw an error!
    error('No nodes with a time less than the most future target!');
end
candidateNodes = allNodes(:, origNodeID);
candidateNodeWeights = allNodeWeights(:, origNodeID);

% BIG PROBLEM! The first find is changing the underlying node IDs.. need to
% translate between the different set of node IDs!

% Probabilistically select the most fit node to expand
[Xcurr,Wcurr,idCand] = selsus(candidateNodes,candidateNodeWeights,1);

% Above picked from the subset of all possible nodes. So we need to
% translate that back to the original full set of nodes.
IDcurr = origNodeID(idCand);

% Find either an unsensed target or target with min sensing
% effectiveness.
Tj = selectUnsensedTarget(G, IDcurr, targets, minEff);
if ( isempty(Tj) )
    % Should never get here..
    error('Could not select an unsensed target!');
end

% =========================================================================
% generate(Tj, N, ngen)
%
%   Generate a set of nodes for target Tj. Employs the user's  generation 
%   function to do all the heavy lifting.
% =========================================================================
function Vj = generate(Tj, N, ngen)
n = 1;
while (n <= N)
    Vtemp = ngen(Tj);
    if ( ~isempty(Vtemp) )
        Vj(:,n) = Vtemp;
        n = n + 1;
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
function [bestBranch, bestBranchWeights, bestBranchEff, bestBranchScore] = evaluate(branches, weights, targets, neval, doCollisionCheck, ccheck)
% Get the sizes of things and allocate some space
numCandidates = size(branches, 3);
numNodes = size(branches, 2) - 1;
numTargets = size(targets,2);
allBranchEff = zeros(numNodes, numTargets, numCandidates);
avgBranchEff = zeros(1,numCandidates);

% Compute the average sensing effectiveness for each branch. I am purposely
% delaying collision checking here since it is assumed that will be a
% computationally intensive task.
for c = 1:numCandidates
    % Select only the intermediate nodes and the generate vantage point. We
    % have already computed the sensing effectiveness of the root node in a
    % prior iteration. No need to reinclude it here.
    branch = branches(:,2:end,c);
    
    % Get the number of nodes for this branch and allocate some temporary
    % storage.
    tempEff = zeros(numNodes,numTargets);
    for n = 1:numNodes
        tempEff(n,:) = nodeSensingEffectiveness(branch(:,n), targets, neval);        
    end
    avgBranchEff(c) = mean(max(tempEff));
    allBranchEff(:,:,c) = tempEff;
end

% Sort candidates by their average branch effectiveness
[ignore,ind] = sort(avgBranchEff, 2, 'descend');

% Default values are empty and 0 for the returned branch and score
% respectively.
new_branch = [];
score = 0;

% If we aren't doing collision checks, simply return the branch that has
% the best mean score.
if (doCollisionCheck)
    % Start with the best branch and work to the worst. First branch
    % that is collision free will be the one we go with. By delaying the
    % collision check and doing it in this fashion, hopefully we will save
    % a few CPU cycles.
    for c = ind
        branch = branches(:,2:end,c);
        
        isBranchCollisionFree = true; % Assume yes to start with.
        for n = 1:numNodes
            if ( ccheck(branch(:,n)) )
                isBranchCollisionFree = false; % If collision function returns true, branch has a collision.
                break;
            end
        end
        
        if (isBranchCollisionFree)
            bestBranch = branch;
            bestBranchEff = allBranchEff(:,:,c);
            bestBranchScore = avgBranchEff(c);
            bestBranchWeights = weights(c,:);
            break;
        end
    end
else
    bestBranch = branches(:,2:end,ind(1));
    bestBranchEff = allBranchEff(:,:,ind(1));
    bestBranchScore = avgBranchEff(ind(1));
    bestBranchWeights = weights(ind(1),:);
end

% =========================================================================
% solution_check(G, branchIDs, numTargets, minEff)
%
%   Checks the newly created branch to see if we have achieved the sensing
%   requirements. Again the requirement for success is simply whether we
%   have achieved the minimum sensing requirements for all the targets.
% =========================================================================
function [path, pathEff, pathDist] = solution_check(G, branchIDs, numTargets, minEff)

% Check to see if the recently added branch results in a successful path to
% the goal.
[pathCandidate, pathCandidateDist, pathCandidateEff] = get_shortest_path(G, branchIDs(end));
numSensed = length( find( pathCandidateEff > minEff ) );

if (numTargets == numSensed)
    path = pathCandidate;
    pathDist = pathCandidateDist;
    pathEff = pathCandidateEff;
else
    path = [];
    pathDist = inf;
    pathEff = [];
end

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
    [G,new_ids(n)] = add_node(G, newBranch(:,n), branchEff(n,:));
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