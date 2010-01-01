% =========================================================================
% extend(Vj, Xi, lp)
%
%   Computes paths from the currently selected node, X, to the set of 
%   generated vantage points, Vj. Employs a user supplied local planning
%   function to determine if a feasible path exists.
% =========================================================================
function [path_candidates, path_weights] = extend(X, Vj, lp);
numVantagePts = size(Vj,2);
path_candidates = [];
path_weights = [];
c = 1;

for n = 1:numVantagePts
    % Determine if a feasable path exists.
    path = lp(X, Vj(:,n));
    
    if ( ~isempty(path.ew) )
        path_candidates(:,:,c) = path.xi;
        path_weights(c,:) = path.ew;
        c = c + 1;
    end
end