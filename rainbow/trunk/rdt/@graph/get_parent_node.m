function parent_id = get_parent_node(G, child_id)
% get_parent_node(G, child_id) returns the parent of child_id in the tree G
parent_id = find(G.EdgeWeights(:,child_id) > 0);
if (length(parent_id) > 1)
    warning('More than one parent node found. Tree may be corrupted!');
end