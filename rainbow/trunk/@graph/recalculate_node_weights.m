function G = recalculate_node_weights(G, minEff, alpha)
% Recompute all shortest paths
G = compute_shortest_paths(G);

% Get max path distance
numRootNodes = length(G.root_ids);
[ignore, numNodes] = size(G.node_data);
D = reshape(G.pathDistances, numRootNodes*numNodes, 1);
ind = find( isfinite(D) );
dmax = max( max(D(ind)), 1e-6);

% Reset graph weights
G.node_weights = zeros(1, numNodes);

% Recompute weights
for id = 1:numNodes
    % Get path information
    [path, pathDist, pathEff] = get_shortest_path(G, id);

    % NEW WAY - Based on single target traveling in time
    w1 = 0;
    if (length(path) == 1)
        w1 = (pathEff/minEff)^2;
    else
        t = path(end,:);
        F = trapz(t,pathEff);
        Fmin = (t(end) - t(1)) * minEff;
        w1 = (F/Fmin)^2;
    end
    w2 = (1 - (pathDist/dmax))^2;
    G.node_weights(id) = sqrt(alpha*w1 + (1-alpha)*w2);
end