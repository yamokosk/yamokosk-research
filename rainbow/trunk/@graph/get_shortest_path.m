function [path, dist, eff] = get_shortest_path(G, id)

% Using precomputed shortest path data. Could be invalid if the underlying
% graph stucture has been changed since the last call to
% compute_shortest_paths()
distCheck = G.pathDistances(id,:);

[ignore,ind] = sort(distCheck,2,'ascend');

dist = G.pathDistances(id,ind(1));
path = path_from_pred(G.pathPredecessors(:,ind(1)),id);

if (nargout > 2)
    if ( length(path) > 1 )
        eff = max(G.node_effectiveness(path,:));
    else
        eff = G.node_effectiveness(id,:);
    end
end