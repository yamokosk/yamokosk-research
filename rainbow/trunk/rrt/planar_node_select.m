function colID = planar_node_select(query, states, Wv, Prob)
% query - query states in num_states x num_query_states matrix
% states - currently known states
% Wv - weights for known states
% Prob - RDT problem structure
[nrows, nquery] = size(query);
[nrows, nstates] = size(states);

% Compute normalized distance matrix
%   dChi = 2*dX ./ range
Q = reshape(query, [nrows, 1, nquery]);
Q = repmat(Q, [1, nstates, 1]);
X = repmat(states, [1, 1, nquery]);
R = repmat(Prob.x_range, [1, nstates, nquery]);
dX = X - Q;
dChi = (2 * dX) ./ R;

% Get closest state IDs
criterion = 2/1000; % 2ms
indices = [1:nstates]';
colID = zeros(1, nquery);

for n = 1:nquery
    % Only interested in states that are greater than criterion
    dt_real = squeeze( dX(1,:,n) );
    ind = find( abs(dt_real) > criterion );

    % Compute distances two different ways
    %   With time included
    %D = sqrt(sum( squeeze(dChi(:,:,n)).^2 ))';
    %   Without time included
    D = sqrt(sum( squeeze(dChi(2:end,:,n)).^2 ))';
    D_sorted = sortrows( [D(ind), indices(ind)] );
    
    if ( ~isempty(D_sorted) )
        colID(n) = D_sorted(1,2);
    else
        colID(n) = -1;
    end
end