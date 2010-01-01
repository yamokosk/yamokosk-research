function G = recalculate_node_weights(G, minEff, alpha)

global tspan;

% Recompute all shortest paths
G = compute_shortest_paths(G);

% Get max path distance
rootIds = get_root_ids(G);
numRootNodes = length(rootIds);
D = reshape(G.ShortestPathDistances, numRootNodes*G.NodeCount, 1);
ind = find( isfinite(D) );
dmax = max( max(D(ind)), 1e-6);
dmean = max( mean( D(ind) ), 1e-6);
Fmin = minEff * (tspan(2) - tspan(1));

% Pre-compute all nodes' sensing effectiveness
F_hat = zeros(1,G.NodeCount);
d_hat = zeros(1,G.NodeCount);
for id = 1:G.NodeCount
	% Get path information
    [pathIds, d, pathEff] = get_shortest_path(G, id);

    % NEW WAY - Based on single target traveling in time
    if (length(pathIds) > 1)
        pathNodes = G.NodeData(:,pathIds);
        t = pathNodes(end,:);
        F_hat(1,id) = trapz(t,pathEff);
		d_hat(1,id) = (dmax - d);
	else
		% length(pathIds) < 1 indicates that we are dealing with just a
		% single node. Or otherwise the node has no path attached to it. So
		% to weight this we are going to just use the node's sensing
		% effectiveness + 1 for the path length.
		%F_hat(1,id) = pathEff;
        F_hat(1,id) = 0;
		d_hat(1,id) = 1;
	end
end

F_hat_bar = mean(F_hat);
F_hat_std = std(F_hat);
if ( F_hat_std == 0 )
    F_hat_std = 1;
end
w_F = (F_hat - F_hat_bar)/F_hat_std;

d_hat_bar = mean(d_hat);
d_hat_std = std(d_hat);
if ( d_hat_std == 0 )
	d_hat_std = 1;
end
w_d = (d_hat - d_hat_bar)/d_hat_std;

w = alpha*w_F + (1-alpha)*w_d;
G.NodeWeights(1,1:G.NodeCount) = w;