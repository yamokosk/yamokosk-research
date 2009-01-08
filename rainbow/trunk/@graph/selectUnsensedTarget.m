function Ti = selectUnsensedTarget(G, id, targets)
minEff = 0.85;
[d pred] = shortest_paths(G.connectivity, 1, 'edge_weight', edge_weight_vector(G.connectivity, G.edge_weights), 'target', id);

path = path_from_pred(pred,id);
if ( length(path) > 1 )
    pathEff = max(G.node_effectiveness(path,:));
else
    pathEff = G.node_effectiveness(id,:);
end

% First find all targets with times greater than current node
t_node = G.node_data(end,id);
futureTargetIDs = find( targets(end,:) > t_node );

if ( isempty(futureTargetIDs) )
    Ti = 0;
else
    % Of these future targets, find either the first one with a sensed rating
    % less than minEff or simply the least sensed one.
    futurePathEff = pathEff(futureTargetIDs);

    ind = find( futurePathEff <= minEff );

    if ( ~isempty(ind) )
        Ti = futureTargetIDs(ind(1));
    else
        ind = find( futurePathEff == min(futurePathEff) );
        Ti = futureTargetIDs(ind);
    end
end