function fitness = distance_metric(state, target)

% Note to me:   state specified as [x; y; vx; vy] 
%               target specified as [x; y; ux; uy]

% Distance metric calculation: Calculate desired vantage point as -1 units 
% in the desired direction from the target location.
c = target(1:2,:) + target(3:4,:);
fitness = gaussian_rbf(state(1:2,:), c, 5);