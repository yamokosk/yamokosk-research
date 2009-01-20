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

nnodes = size(nodes,2);
M = size(G.node_data,2);
ind = [1:nnodes] + M;
G.node_data = [G.node_data, nodes];
G.node_weights = [G.node_weights, 0];
G.node_effectiveness = [G.node_effectiveness; eff];

% Deal with the connectivity and weight matrices
[i,j,s] = find(G.connectivity);
G.connectivity = sparse(i,j,s,M+nnodes,M+nnodes);

[i,j,s] = find(G.edge_weights);
G.edge_weights = sparse(i,j,s,M+nnodes,M+nnodes);

if (isRootNode)
    G.root_ids = [G.root_ids, ind];
end