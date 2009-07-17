function G = compute_shortest_paths(G)

% Use MatlabBGL toolbox to calculate shortest paths from node 1 to all
% other nodes. Assume this is computationally intensive step so we will
% only do this as required.
if (G.ShortestPathsInvalid)
    rootids = get_root_ids(G);
    rootNodeCount = length(rootids);
    E = get_connectivity(G);
    We = G.EdgeWeights(1:G.NodeCount,1:G.NodeCount);
    We_vec = edge_weight_vector(E, We);

    %[origNodeCount,origRootNodeCount] = size(G.ShortestPathDistances);
    
%     % The 'hint' is a root node for which we want to recalculate the path distances.
%     % If no hint or root node is passed in, we will calculate the shortest paths
%     % for all root nodes. Otherwise we will do it just for that one root
%     % node.
%     if (( nargin < 2 ) || ( origNodeCount == 0 ))
        G.ShortestPathDistances = zeros(G.NodeCount,rootNodeCount);
        G.ShortestPathPredecessors = zeros(G.NodeCount,rootNodeCount);
        for n = 1:rootNodeCount
            [d, pred] = shortest_paths(E, rootids(n), 'edge_weight', We_vec);
            G.ShortestPathDistances(:,n) = d;
            G.ShortestPathPredecessors(:,n) = pred;
        end
        G.ShortestPathsInvalid = false;
%     else
%         index = find( hint == rootids );
%         
%         oldShortestPathDistances = G.ShortestPathDistances;
%         oldShortestPathPredecessors = G.ShortestPathPredecessors;
%         
%         G.ShortestPathDistances = inf*ones(G.NodeCount,rootNodeCount);
%         G.ShortestPathPredecessors = zeros(G.NodeCount,rootNodeCount);
%         
%         G.ShortestPathDistances(1:origNodeCount, 1:origRootNodeCount) = oldShortestPathDistances;
%         G.ShortestPathPredecessors(1:origNodeCount, 1:origRootNodeCount) = oldShortestPathPredecessors;
%         
%         [d, pred] = shortest_paths(E, hint, 'edge_weight', We_vec);
%         
%         G.ShortestPathDistances(:,index) = d;
%         G.ShortestPathPredecessors(:,index) = pred;
%     end
end
        
    