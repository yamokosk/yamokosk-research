function conn = get_connectivity(G)
[i,j] = find( G.EdgeWeights > 0 );
conn = sparse(i,j,1,G.NodeCount,G.NodeCount);