function G = recalculate_node_weights(G, minEff, sig)
% Recompute all shortest paths
G = compute_shortest_paths(G);

% Get max path distance
dmax = max(max(G.pathDistances),1e-6);

% Get some size information
numTargets = size(G.node_effectiveness,2);
numNodes = size(G.node_data,2);

% Reset graph weights
G.node_weights = zeros(1, numNodes);

% Recompute weights
for id = 1:numNodes
    % Get path information
    [path, pathDist, pathEff] = get_shortest_path(G, id);
    
    % Number of sensed targets are those with an effectiveness greater than
    % the minimum allowable.
    numSensed = length( find( pathEff > minEff ) );
    
    % From Kehoe's thesis. Proposed node weight calculation:
    Wsquared = numSensed^2 + (numTargets * ((pathDist/dmax) - 1) - sig)^2;
    G.node_weights(id) = sqrt(Wsquared);
end