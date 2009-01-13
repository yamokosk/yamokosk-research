function G = compute_shortest_paths(G)

% Use MatlabBGL toolbox to calculate shortest paths from node 1 to all
% other nodes. Assume this is computationally intensive step so we will
% only do this as required.
if (G.pathsInvalid)
    [d pred] = shortest_paths(G.connectivity, 1, 'edge_weight', edge_weight_vector(G.connectivity, G.edge_weights));
    G.pathDistances = d;
    G.pathPredecessors = pred;
    G.pathsInvalid = false;
end