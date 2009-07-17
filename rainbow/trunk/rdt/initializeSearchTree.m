% =========================================================================
% initializeSearchTree(x0, targets, neval)
%
%   Performs some simple initialization steps for the planner, including:
%       * Initialize output function
%       * Initialize solution data structure, if needed
% =========================================================================
function G = initializeSearchTree(x0, target, neval)
[dof, numStartingNodes] = size(x0);
G = graph(dof);
Tvar=diag(target.variance);
for n = 1:numStartingNodes
    Tbar = ppval(target.pp, x0(end));
    eff = nodeSensingEffectiveness(x0(:,n), Tbar, Tvar, neval);
    G = add_node(G, x0(:,n), eff, true);
end