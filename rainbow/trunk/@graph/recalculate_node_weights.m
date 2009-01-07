function G = recalculate_node_weights(G)
sig = -31;
minEff = 0.85;

[d pred] = shortest_paths(G.connectivity, 1, 'edge_weight', edge_weight_vector(G.connectivity, G.edge_weights));
dmax = max(max(d),1e-6);

numTargets = size(G.node_effectiveness,2);
for id = 1:length(d)
    path = path_from_pred(pred,id);
	%pathEff = [];
    if ( length(path) > 1 )
        pathEff = max(G.node_effectiveness(path,:));
    else
        pathEff = G.node_effectiveness(id,:);
    end
    numSensed = length( find( pathEff > minEff ) );
    Wsquared = numSensed^2 + (numTargets * ((d(id)/dmax) - 1) - sig)^2;
    G.node_weights(id) = sqrt(Wsquared);
end