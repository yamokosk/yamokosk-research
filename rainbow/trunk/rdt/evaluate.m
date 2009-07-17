% =========================================================================
% evaluate(branches, targets, doCollisionCheck, ccheck)
%
%   Evaluates the branch candidates and returns the branch which has the
%   greatest mean sensing effectiveness and that is also collision free (if
%   doCollisionCheck is true). Note that the collision check is actually
%   delayed until after the mean sensing effectiveness scores are
%   calculated. This is done in hopes of reducing the number of collision
%   checks we need to actually perform.
% =========================================================================
function [branchID,bestBranchEff] = evaluate(branches, target, neval)

% Get the sizes of things and allocate some space
numCandidates = size(branches, 3);
numNodes = size(branches, 2);
%allBranchEff = zeros(numNodes, numTargets, numCandidates);
avgBranchEff = zeros(1,numCandidates);

% Compute the average sensing effectiveness for each branch. I am purposely
% delaying collision checking here since it is assumed that will be a
% computationally intensive task.
Tvar=diag(target.variance);
F = zeros(numCandidates,1);
f = zeros(numNodes,numCandidates);
for c = 1:numCandidates
    % Select only the intermediate nodes and the generate vantage point. We
    % have already computed the sensing effectiveness of the root node in a
    % prior iteration. No need to reinclude it here.
    branch = branches(:,1:end,c);
    
    % NEW NEW METHOD.. using unscented transform to estimate node
    % evaluation statistics
    t_branch = branch(end,:)';
    Tbar = ppval(target.pp, t_branch)';
    for n = 1:numNodes
        f(n,c) = nodeSensingEffectiveness(branch(:,n), Tbar(:,n), Tvar, neval);
    end
    F(c) = trapz(t_branch,f(:,c));
end

% Sort candidates by their average branch effectiveness
%[ignore,ind] = sort(avgBranchEff, 2, 'descend');
[ignore,ind] = sort(F, 1, 'descend');

branchID = ind(1);
bestBranchEff = f(:,ind(1))';