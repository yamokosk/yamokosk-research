% =========================================================================
% add_branch(G, rootID, newBranch, branchEff)
%
%   Really a utility function to automate the addition of a collection of
%   nodes to the tree. The nodes are assumed to be in the order which they
%   should be connected.
% =========================================================================
function [G, ids] = add_branch(G, rootID, newBranch, branchWeights, branchEff)

numNewNodes = size(newBranch,2);

% Add the nodes to the graph
[G, ids] = add_node(G, newBranch, branchEff);

% Make all the new connections
G = add_edge(G, rootID, ids(1), branchWeights(1));
for n = 2:1:numNewNodes
    G = add_edge(G, ids(n-1), ids(n), branchWeights(n));
end

%rootId = get_root_id(G, rootID);

% Force a recomputation of all shortest paths since we have just added a
% new branch.
G = compute_shortest_paths(G);