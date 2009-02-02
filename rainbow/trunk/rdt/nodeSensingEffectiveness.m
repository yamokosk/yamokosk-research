function neff = nodeSensingEffectiveness(node, targets, neval)
t_node = node(end);
t_targets = targets(end,:);
target_interp = interp1(t_targets,targets(1:end-1,:)',t_node,'spline');

neff = neval(node, [target_interp'; t_node]);

% nt=size(targets,2); 
% neff=zeros(1,nt); 
% 
% diff_t = abs(t_targets - t_node);
% ind = find( diff_t == min(diff_t) );
% 
% neff(ind) = effectiveness;