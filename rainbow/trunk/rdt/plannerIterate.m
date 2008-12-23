function Prob = plannerIterate(pathLengthBound, Prob)
% plannerIterate - Performs one iteration of the vantage point planning
% algorithm.
%
%   Note! This function should never be called by the user directly. It is
%   meant to be called by the function plannerSolve.

% Get copy of current solution
solution = Prob.solution;

% Generate a set of vantage points for target j
Vp = Prob.f.ngen(i, Prob);
numVantagePts = size(Vp,2);

% Find the nearest neighbors associated with those vantage points
ID = Prob.f.nsel(Vp, G.V, G.Wv, Prob);
ind = find(ID > 0);
Xi = G.V(ID);
Vp = Xr(:,ind);

% Create a branch from the currNode to all the generated vantage points
raw_scores = zeros(numVantagePts, Prob.number_targets);
for v = 1:numVantagePts
    % Determine if a feasable path exists.
    [path,flag,msg] = Prob.f.lp(Xi(:,v), Vp(:,v), Prob);
    
    if ( ~isempty(path) )
        % For each intermediate node returned by the planner, evaluate its
        % sensing performance. Returns a Ni x K matrix of scores
        path_scores = f.neval( path(:,2:end-1), Prob );
        raw_scores(v,:) = mean( path_scores );
    end
end

% Sort the sum of the raw scores from highest to lowest. Then starting with
% the best path do collision detection until we find a collision free path
% that is collision free.
[path_scores_sorted, path_index] = sort( sum(path_scores, 2), 1, 'descend' );
best_branch = 0;
for ind=path_index
    if ( isPathCollisionFree( branches(:,2:end, ind), Prob ) )
        best_branch = ind;
        break;
    end
end

if (best_branch ~= 0)
    % We have a good, new branch. Add it to the solution.
    % The first node should already be in the tree. So first evaluate and add
    % the new nodes
    NewNodes = Path(:,2:end);
    NewNodeWeights = Prob.node_evaluate(NewNodes, Prob);
    [G_new,NewNodeIDs] = add_node(G_new, NewNodes, NewNodeWeights);

    % Next make an edge between existing node in the tree and our first new
    % node.
    %G_new = add_edge(G_new, ID, NewNodeIDs(1), EdgeWeights(1));
    IDWeight = G_new.Wv(ID);
    G_new = add_edge(G_new, ID, NewNodeIDs(1), (IDWeight + NewNodeWeights(1))/2 );

    % Then use a for loop to add the rest of the edges
    nedges = length(NewNodeIDs);
    for i = 2:nedges
        %G_new = add_edge(G_new, NewNodeIDs(i-1), NewNodeIDs(i), EdgeWeights(i));
        G_new = add_edge(G_new, NewNodeIDs(i-1), NewNodeIDs(i), (NewNodeWeights(i) + NewNodeWeights(i-1))/2 );
    end 
end