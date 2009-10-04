function G = add_edge(G,node_from_id,node_to_id,edge_weight)
% add_edge(G, ID1, ID2, weight) adds a directional, weighted edge
% from G.V(ID1) to G.V(ID2).
if (node_from_id == node_to_id)
    warning('Self-referencing edge being added to graph.');
end

if (edge_weight <= 0)
    warning(['Illegal weight (' num2str(edge_weight) ') between ' num2str(node_from_id) ' and ' num2str(node_to_id) '. Weights must be strictly positive.']);
    edge_weight = inf;
end

% Record connection in the sparse edge weight matrix
G.EdgeWeights(node_from_id,node_to_id) = edge_weight;

% Adjust the leaf IDs flags
G.LeafIds(1,node_from_id) = 0;
G.LeafIds(1,node_to_id) = 1;

% Record that the from node was 'extended'
G.NodeExtendedCount(1,node_from_id) = G.NodeExtendedCount(1,node_from_id) + 1;

% Invalidate the shortest paths.. new edge means new paths.
G.ShortestPathsInvalid = true;