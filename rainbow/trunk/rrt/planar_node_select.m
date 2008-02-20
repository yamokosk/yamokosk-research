function colID = planar_node_select(xq, V, Wv)
[ns,n] = size(V);
Xq = repmat(xq,1,n);
dV = V - Xq;

dist = sqrt(sum(dV.^2))';
indices = 1:n;
sorted_dist = sortrows([dist, indices']);
colID = sorted_dist(1,2);