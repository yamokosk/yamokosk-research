function children = get_child_nodes(G, node_id)
% get_child_nodes(G, v_index) returns the children of node_id in the tree G
children = find(G.connectivity(node_id,:) > 0);