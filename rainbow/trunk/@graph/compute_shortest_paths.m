function G = compute_shortest_paths(G)

% Use MatlabBGL toolbox to calculate shortest paths from node 1 to all
% other nodes. Assume this is computationally intensive step so we will
% only do this as required.
if (G.pathsInvalid)
    [ignore,numNodes] = size(G.node_data);
    numRootNodes = length(G.root_ids);
    G.pathDistances = zeros(numNodes,numRootNodes);
    G.pathPredecessors = zeros(numNodes,numRootNodes);
    for n = 1:length(G.root_ids)
        [d pred] = shortest_paths(G.connectivity, G.root_ids(n), 'edge_weight', edge_weight_vector(G.connectivity, G.edge_weights));
        G.pathDistances(:,n) = d;
        G.pathPredecessors(:,n) = pred;
    end
    G.pathsInvalid = false;
end