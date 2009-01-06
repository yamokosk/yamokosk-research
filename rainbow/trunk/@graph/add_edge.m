function G = add_edge(G,node_from_id,node_to_id,edge_weight)
% add_edge(G, ID1, ID2, weight) adds a directional, weighted edge
% from G.V(ID1) to G.V(ID2).
if (node_from_id == node_to_id)
    warning('Self-referencing edge being added to graph.');
end
G.connectivity(node_from_id,node_to_id) = 1;
G.edge_weights(node_from_id,node_to_id) = edge_weight;