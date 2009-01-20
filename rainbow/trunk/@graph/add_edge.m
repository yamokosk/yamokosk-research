function G = add_edge(G,node_from_id,node_to_id,edge_weight)
% add_edge(G, ID1, ID2, weight) adds a directional, weighted edge
% from G.V(ID1) to G.V(ID2).
if (node_from_id == node_to_id)
    warning('Self-referencing edge being added to graph.');
end
G.connectivity(node_from_id,node_to_id) = 1;

% Edge weight is the length of time the from node to the to node
dt = G.node_data(end,node_to_id) - G.node_data(end,node_from_id);

if (nargin < 4)
    G.edge_weights(node_from_id,node_to_id) = dt;
    %G.edge_weights(node_to_id,node_from_id) = dt;
else
    G.edge_weights(node_from_id,node_to_id) = edge_weight;
    %G.edge_weights(node_to_id,node_from_id) = edge_weight;
end

% Invalidate the shortest paths.. new edge means new paths.
G.pathsInvalid = true;