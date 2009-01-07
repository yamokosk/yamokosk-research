function Ti = selectUnsensedTarget(G, id)
minEff = 0.85;
[d pred] = shortest_paths(G.connectivity, 1, 'edge_weight', edge_weight_vector(G.connectivity, G.edge_weights), 'target', id);

path = path_from_pred(pred,id);
if ( length(path) > 1 )
    pathEff = max(G.node_effectiveness(path,:));
else
    pathEff = G.node_effectiveness(id,:);
end

ind = find( pathEff <= minEff );
Ti = ind(1);