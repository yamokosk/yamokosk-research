function neff = nodeSensingEffectiveness(node, Tbar, Tvar, neval)

fun = @(y)neval(node, y);
neff = ut_transform(Tbar,Tvar,fun,[],[],[],[],1);

% nt=size(targets,2); 
% neff=zeros(1,nt); 
% 
% diff_t = abs(t_targets - t_node);
% ind = find( diff_t == min(diff_t) );
% 
% neff(ind) = effectiveness;