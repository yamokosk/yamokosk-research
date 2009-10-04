function G = resize_internal_storage(G, nodeCount, force)

if ( nargin < 3 )
    force = 0;
end

% Get current sizes of things
nc = G.NodeCount;
[dof,junk] = size(G.NodeData);
memSize = G.MemSize;

% Check to see if resizing the internal storage is even necessary. If so,
% do it! Otherwise just return with out touching anything.
if ( (( nodeCount + nc ) > memSize) || force )
    % Local copy current data
    nodeData = G.NodeData(:,1:nc);
    nodeWeights = G.NodeWeights(1,1:nc);
    nodeEffectiveness = G.NodeEffectiveness(1,1:nc);
    edgeWeights = G.EdgeWeights(1:nc,1:nc);
    nodeVisitedCount = G.NodeVisitedCount(1,1:nc);
    nodeExtendedCount = G.NodeExtendedCount(1,1:nc);
    rootIds = G.RootIds(1,1:nc);
    leafIds = G.LeafIds(1,1:nc);
    
    % Resize internal class storage
    memSize = memSize + 2 + nodeCount;
    G.MemSize = memSize;
    G.NodeData = zeros(dof,memSize);
    G.NodeWeights = zeros(1,memSize);
    G.NodeEffectiveness = zeros(1,memSize);
    G.EdgeWeights = sparse(memSize,memSize);
    G.NodeVisitedCount = sparse(1,memSize);
    G.NodeExtendedCount = sparse(1,memSize);
    G.RootIds = sparse(1,memSize);
    G.LeafIds = sparse(1,memSize);
    
    % Copy local data back into new storage
    G.NodeData(:,1:nc) = nodeData;
    G.NodeWeights(1,1:nc) = nodeWeights;
    G.NodeEffectiveness(1,1:nc) = nodeEffectiveness;
    G.EdgeWeights(1:nc,1:nc) = edgeWeights;
    G.NodeVisitedCount(1,1:nc) = nodeVisitedCount;
    G.NodeExtendedCount(1,1:nc) = nodeExtendedCount;
    G.RootIds(1,1:nc) = rootIds;
    G.LeafIds(1,1:nc) = leafIds;
end
