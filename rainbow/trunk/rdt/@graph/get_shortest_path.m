function [path, dist, eff] = get_shortest_path(G, id)

% Using precomputed shortest path data. Could be invalid if the underlying
% graph stucture has been changed since the last call to
% compute_shortest_paths()
distCheck = G.ShortestPathDistances(id,:);

[ignore,ind] = sort(distCheck,2,'ascend');

dist = G.ShortestPathDistances(id,ind(1));
path = path_from_pred(G.ShortestPathPredecessors(:,ind(1)),id);

if (nargout > 2)
    if ( length(path) > 1 )
        eff = G.NodeEffectiveness(path);
    else
        eff = G.NodeEffectiveness(id);
    end
end