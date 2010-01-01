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
function [branchID,bestBranchEff] = evaluate(branches, target, neval, doCollisionCheck, ccheck)

% Get the sizes of things and allocate some space
numCandidates = size(branches, 3);
numNodes = size(branches, 2);
%allBranchEff = zeros(numNodes, numTargets, numCandidates);
avgBranchEff = zeros(1,numCandidates);

% Compute the average sensing effectiveness for each branch. I am purposely
% delaying collision checking here since it is assumed that will be a
% computationally intensive task.
%Tvar=diag(target.variance);
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
    tvar = ppval(target.variance, t_branch)';
    for n = 1:numNodes
        Tvar = diag(tvar(:,n));
        f(n,c) = nodeSensingEffectiveness(branch(:,n), Tbar(:,n), Tvar, neval);
    end
    F(c) = trapz(t_branch,f(:,c));
end

% Sort candidates by their average branch effectiveness
%[ignore,ind] = sort(avgBranchEff, 2, 'descend');
[ignore,ind] = sort(F, 1, 'descend');
branchID = [];
bestBranchEff = [];
if (doCollisionCheck)
    for c = 1:numCandidates
        branch = branches(:,2:end,ind(c));
        nnodes = size(branch,2);
        branchInCollision = false;
        
        for n = 1:nnodes
            if ccheck(branch(:,n))
                branchInCollision = true;
                break;
            end
        end
        
        if ( ~branchInCollision )
            branchID = ind(c);
            bestBranchEff = f(:,ind(c))';
            break;
        end
    end
else
    branchID = ind(1);
    bestBranchEff = f(:,ind(1))';
end