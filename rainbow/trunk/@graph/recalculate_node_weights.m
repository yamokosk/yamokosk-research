function G = recalculate_node_weights(G, minEff, alpha)

global tspan;

% Recompute all shortest paths
G = compute_shortest_paths(G);

% Get max path distance
numRootNodes = length(G.root_ids);
[ignore, numNodes] = size(G.node_data);
D = reshape(G.pathDistances, numRootNodes*numNodes, 1);
ind = find( isfinite(D) );
dmax = max( max(D(ind)), 1e-6);
dmean = mean( D(ind) );

% Reset graph weights
G.node_weights = zeros(1, numNodes);

% Recompute weights
for id = 1:numNodes
    % Get path information
    [path, pathDist, pathEff] = get_shortest_path(G, id);

    % NEW WAY - Based on single target traveling in time
    if (length(path) > 1)
        t = path(end,:);
        F = trapz(t,pathEff);
        Fmin = minEff * (tspan(2) - tspan(1));
        
        G.node_weights(id) = sqrt(alpha*(F/Fmin)^2 + (1-alpha)*((dmax - pathDist)/dmean)^2);
    else
        G.node_weights(id) = pathEff;
    end
end