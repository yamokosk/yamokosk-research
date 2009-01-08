function Prob = plannerSolve(Prob, targets)
% plannerSolve - Solves the vantage point planning problem for my thesis!
%
%   Prob = plannerSolve(Prob) attempts to solve the vantage point planning
%   problem for a problem domain specified in Prob. See plannerAssign for 
%   information on constructing a proper Problem data structure.
%
%   Assumptions
%   *   Continuous time dynamic systems.   

% Create some local variables for ease of readability
num_iterations = Prob.iterations;
numTargets = size(targets,2);
outfun = Prob.func_handles.outfun;
goalfun = Prob.func_handles.goalfun;
neval = @(x) Prob.func_handles.neval(x, targets, Prob.userdata);

% Do some initialization stuff
outfun('init', '', Prob);
sol = struct();
if isempty(Prob.solution)
    sol = initialize(Prob, targets);
else
    sol = Prob.solution;
end    

% Create local solution variables
best_path_length = sol.bestPathLength;
G = sol.connectivityGraph;
completeNodes = sol.completeNodes;
completeLengths = sol.completeLengths;
completeScores = sol.completeScores;

% Now just do the specified number of iterations
for iter = 1:num_iterations
    % Recalculate node weights
    G = recalculate_node_weights(G);
    
    j = 0;
    while (1)
        % Probabilistically select the most fit node
        [Xcurr,Wcurr,IDcurr] = selsus(G.V,G.Vw,1);

        % Find either an unsensed target or target with min sensing
        % effectiveness
        j = selectUnsensedTarget(G, IDcurr, targets);
        if ( j > 0 )
            Tj = targets(:,j);
            break;
        end
    end
    
    % Generate a set of vantage points for target j
    Nj = 5;
    Vj = zeros(length(Xcurr),5);
    for n = 1:Nj
        Vj(:,n) = generate(Prob, Tj);
    end
    
    % Extend the tree to one of the generated nodes
    branch_candidates = extend(Prob, Xcurr, Vj);
    
    % Compute shortest path from x0 to all other nodes.
    [new_branch, score] = evaluate(Prob, branch_candidates, targets);
    
    % Add this new branch to the tree
    if ( ~isempty(new_branch) )
        G = add_branch(Prob, G, IDcurr, new_branch, targets);
            
        % Check to see if any of the paths result is meeting our goal.
        %[nodes, len, scores] = solutionCheck(G, best_path_length);
        [nodes, len, scores] = solutionCheck(G, inf);
        
        [newCompleteNodes, I] = setdiff(nodes, completeNodes);

        if ( ~isempty(newCompleteNodes) )
            completeNodes = [completeNodes; newCompleteNodes];
            completeLengths = [completeLengths; len(I)];
            completeScores = [completeScores; scores(I)];
            minLengthSoFar = min( completeLengths );
            
            if ( minLengthSoFar < best_path_length)
                best_path_length = minLengthSoFar;
            end
        else
            %outfun('iter', 'Complete', Prob);
        end
    end
    
    msg = sprintf('%d   p(%f)   s(%f)', iter, best_path_length, max(completeScores));
    outfun('iter', msg, Prob);
end

% Notify output function we are done
outfun('done', '', Prob);

% Copy things back into solution structure
sol.bestPathLength = best_path_length;
sol.connectivityGraph = G;
sol.completeNodes = completeNodes;
sol.completeLengths = completeLengths;
sol.completeScores = completeScores;

Prob.solution = sol;


% =========================================================================
% initialize(Prob)
%
%   Performs some simple initialization steps for the planner, including:
%       * Initialize output function
%       * Initialize solution data structure, if needed
% =========================================================================
function sol = initialize(Prob, targets)
G = graph();
eff = nodeEffectiveness(Prob.func_handles.neval, Prob.x0, targets, Prob.userdata);
G = add_node(G, Prob.x0, eff);
sol = struct('connectivityGraph', G, ...
             'bestPathLength', inf, ...
             'completeNodes', [], ...
             'completeLengths', [], ...
             'completeScores', []);

% =========================================================================
% findMinTarget(targets, Seff)
% =========================================================================
function Ti = findMinTarget(targets, Seff)
ind = find( Seff == min(Seff) );
Ti = targets(:,ind(1));

% =========================================================================
% generate(Prob, Tj)
% =========================================================================
function Vj = generate(Prob, Tj)
Vj = Prob.func_handles.ngen(Tj, Prob.userdata);

% =========================================================================
% select(Prob, Vp, G)
% =========================================================================
function Xi = select(Prob, Vj, G)
ID = Prob.func_handles.nsel(Vj, G.V, G.Vw);
ind = find(ID > 0);
Xi = G.V(ID);

% =========================================================================
% extend(Prob, Vj, Xi, G)
% =========================================================================
function path_candidates = extend(Prob, Xcurr, Vj);
numVantagePts = size(Vj,2);
path_candidates = [];
c = 1;
for n = 1:numVantagePts
    % Determine if a feasable path exists.
    xi = Prob.func_handles.lp(Xcurr, Vj(:,n), Prob.userdata);
    
    if ( ~isempty(xi) )
        path_candidates(:,:,c) = [Xcurr, xi, Vj(:,n)];
        c = c + 1;
    end
end

% =========================================================================
% evaluate(Prob, branch_candidates)
% =========================================================================
function [new_branch, score] = evaluate(Prob, branch_candidates, targets)
numCandidates = size(branch_candidates, 3);
numTargets = size(targets,2);
avgBranchEff = zeros(1,numCandidates);
for c = 1:numCandidates
    branch = branch_candidates(:,2:end,c);
    
    % Delay collision check. First sort the branches by their effectiveness
    numNodes = size(branch,2);
    branchEff = zeros(1,numTargets);
    for n = 1:numNodes
        neff = nodeEffectiveness(Prob.func_handles.neval, branch(:,n), targets, Prob.userdata);        
        branchEff = max(branchEff,neff);
    end
    avgBranchEff(c) = mean(branchEff);
end

% Sort candidates by their average branch effectiveness
[ignore,ind] = sort(avgBranchEff, 2, 'descend');

% Default values are empty and 0 for the returned branch and score
% respectively.
new_branch = [];
score = 0;

% If we aren't doing collision checks, simply return the branch that has
% the best mean score.
if (Prob.doCollisionCheck)
    % Start with the best branch and work to the worst. First branch
    % that is collision free will be the one we go with.
    for c = ind
        branch = branch_candidates(:,2:end,c);
        numNodes = size(branch,2);
        branchCheck = true; % Is branch collision free? Assume yes to start with.
        for n = 1:numNodes
            if ( Prob.func_handles.collisionCheck(branch(:,n),Prob.userdata) )
                branchCheck = false; % If collision function returns true, branch has a collision.
                break;
            end
        end
        
        if (branchCheck == true)
            new_branch = branch;
            score = avgBranchEff(c);
            break;
        end
    end
else
    new_branch = branch_candidates(:,2:end,ind(1));
    score = avgBranchEff(ind(1));
end


% =========================================================================
% nodeEffectiveness(Prob, x, targets)
% =========================================================================
function W = evaluateNode(Prob, x, targets)
% Calculate this node's effectiveness
numTargets = size(targets,2);
Xeff = zeros(1, numTargets);
for n = 1:numTargets
    Xeff(n) = Prob.func_handles.neval(x, targets(:,n), Prob.userdata);
end

W = sqrt(numVisible^2 + (numTargets * ((L / Lmax) - 1) - sig)^2);


% =========================================================================
% nodeEffectiveness(Prob, x, targets)
% =========================================================================
function G = add_branch(Prob, G, rootID, new_branch, targets)
numNewNodes = size(new_branch,2);

% Add the nodes to the graph
new_ids = zeros(1,numNewNodes);
for n = 1:numNewNodes
    neff = nodeEffectiveness(Prob.func_handles.neval, new_branch(:,n), targets, Prob.userdata);
    [G,new_ids(n)] = add_node(G, new_branch(:,n), neff);
end

% Make all the new connections
G = add_edge(G,rootID,new_ids(1));
for n = 2:numNewNodes
    G = add_edge(G,new_ids(n-1),new_ids(n));
end


% =========================================================================
% nodeEffectiveness(Prob, x, targets)
% =========================================================================
function [completeNodes, len, scores] = solutionCheck(G, best_path_length)
minEff = 0.85;
[d pred] = shortest_paths(G.E, 1, 'edge_weight', edge_weight_vector(G.E, G.Ew));

ind = find( d < best_path_length );
numTargets = size(G.Seff,2);
completeNodes = [];
len = [];
scores = [];

for id = ind'
    path = path_from_pred(pred,id);
    if ( length(path) > 1 )
        pathEff = max(G.Seff(path));
    else
        pathEff = G.Seff(id);
    end
    numSensed = length( find( pathEff > minEff ) );

    if (numTargets == numSensed)
        completeNodes = [completeNodes; id];
        len = [len; d(id)];
        scores = [scores; mean(pathEff)];
    end
end

if ( ~isempty(completeNodes) )
    [ignore, I] = sort(len,1,'ascend');
    completeNodes = completeNodes(I);
    len = len(I);
    scores = scores(I);
end
        
