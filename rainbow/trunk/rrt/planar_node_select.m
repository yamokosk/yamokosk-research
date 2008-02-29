function colID = planar_node_select(xq, V, Wv, Prob)
[ns,n] = size(V);
Xq = repmat(xq,1,n);
dV = (V - Xq);

% Only interested in states that are at lease 2 ms away
criterion = 2/1000;
dt_real = ( Prob.x_range(1) * dV(1,:) )  ./ 2;
ind = find(abs(dt_real) > criterion);

% Compute distances
dist = sqrt(sum(dV.^2))';
indices = 1:n;

% Only keep distances that are at least 2 ms away
dist = dist(ind);
indices = indices(ind)';

% Sort to find smallest distance
if (~isempty(dist))
    sorted_dist = sortrows([dist, indices]);
    colID = sorted_dist(1,2);
else
    colID = -1;
end