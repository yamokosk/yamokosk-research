function [G,ind] = add_node(G,nodes,eff,isRootNode)
% add_node(G,N) adds the nodes stored in N to the graph G. It returns the
% new graph as well as the indicies of the new nodes. N is assumed to be a
% matrix where the nodes are stored in a column format.
% add_node(..., Wn) adds the nodes as well as the node weights which are
% stored in row format in Wn. If Wn is not specified, the weights are
% assumed to be 0.

if (nargin < 4)
    isRootNode = false;
end

% Check the incoming data to make sure its the same length.
[dof,nodeCount] = size(nodes);
[StoredDof,junk] = size(G.NodeData);
if ( dof ~= StoredDof )
    error( 'Nodes being added are not the same length as the existing nodes in the graph.' );
end

% Will smartly resize the internal storage. Resizing like this is done to
% hopefully optimize the runtime of the planning algorithm.
G = resize_internal_storage(G, nodeCount);

% Add in the new data
nc = G.NodeCount;
ind = (nc+1):(nc+nodeCount);
G.NodeData(:,ind) = nodes;
G.NodeEffectiveness(1,ind) = eff;
G.NodeCount = G.NodeCount + nodeCount;

% By convention, all nodes start out as leaf nodes. Its not until edges are
% added that a node's leaf status is updated.
G.LeafIds(1,ind) = 1;

if (isRootNode)
    G.RootIds(1,ind) = 1;
end