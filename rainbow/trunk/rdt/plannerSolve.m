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
outfun = Prob.func_handles.outfun;
goalfun = Prob.func_handles.goalfun;

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
%Seff = sol.sensing_effectiveness;

% Now just do the specified number of iterations
for iter = 1:num_iterations
    % Recalculate node weights
    G = recalculate_node_weights(G);
    
    % Probabilistically select the most fit node
    [Xcurr,Wcurr,IDcurr] = selsus(G.V,G.Vw,1);
    
    % Find either an unsensed target or target with min sensing
    % effectiveness
    j = selectUnsensedTarget(G, IDcurr);
    Tj = targets(:,j);
    %Tj = findMinTarget(targets, Seff);
    
    % Generate a set of vantage points for target j
    Vj = generate(Prob, Tj);

    
    
    % Extend the tree to one of the generated nodes
    branch_candidates = extend(Prob, Xcurr, Vj);
    
    % Compute shortest path from x0 to all other nodes.
    [new_branch, score] = evaluate(Prob, branch_candidates);
    
    % Add this new branch to the tree
    G = add_branch(G,new_branch, score);
    
    [d pred] = shortest_paths(g.E,1,'edge_weight',edge_weight_vector(G.E,G.Wv));
    
    % Check to see if any of the paths result is meeting our goal.
    ind = find( d < best_path_length );
    for npath = length(ind):-1:2
        path_candidate = path_from_pred(pred,ind(npath));
        
        % Compute sensing effectiveness
        Se = computeSensingEffectiveness(G.V(path_candidate), targets);
    end
        
        
    if ( goalfun(Prob) )
        completed_paths = current_path;
        if (path_length < best_path_length)
            best_path_length = path_length;
        end 
    else
        outfun('iter', 'Complete', Prob);
    end
end

% Notify output function we are done
outfun('done', '', Prob);


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
             'bestPathLength', inf);

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
function [new_branch, score] = evaluate(Prob, G, IDcurr, branch_candidates, targets)
numCandidates = size(branch_candidates, 3);
numTargets = size(targets,2);
for c = 1:numCandidates
    Xcurr = branch_candidates(:,1,c);
    Xeff = get_node_effectiveness(G,IDcurr);
    branch = branch_candidates(:,2:end,c);
    if ( Prob.func_handles.collisionCheck( branch, Prob.userdata ) )
        numNodes = size(branch,2);
        branchEff = zeros(1,numTargets);
        for n = 1:numNodes
            root_id = getNodeID(G,root);
            branchEff = max(branchEff, nodeEffectiveness(Prob, branch(:,n), targets))
        end
        
    end
end

% Sort the sum of the raw scores from highest to lowest. Then starting with
% the best path do collision detection until we find a collision free path
% that is collision free.
[path_scores_sorted, path_index] = sort( sum(path_scores, 2), 1, 'descend' );
best_branch = 0;
for ind=path_index
%     if ( isPathCollisionFree( branches(:,2:end, ind), Prob ) )
%         best_branch = ind;
%         break;
%     end
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