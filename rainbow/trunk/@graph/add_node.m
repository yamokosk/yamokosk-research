function [G,ind] = add_node(G,N,Wn)
% add_node(G,N) adds the nodes stored in N to the graph G. It returns the
% new graph as well as the indicies of the new nodes. N is assumed to be a
% matrix where the nodes are stored in a column format.
% add_node(..., Wn) adds the nodes as well as the node weights which are
% stored in row format in Wn. If Wn is not specified, the weights are
% assumed to be 0.
nnodes = size(N,2);
ind = [1:nnodes] + size(G.node_data,2);
G.node_data = [G.node_data, N];

if (nargin < 3)
    Wn = zeros(1,nnodes);
end

G.node_weights = [G.node_weights; Wn];