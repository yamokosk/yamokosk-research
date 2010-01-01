% =========================================================================
% solution_check(G, branchIDs, numTargets, minEff)
%
%   Checks the newly created branch to see if we have achieved the sensing
%   requirements. Again the requirement for success is simply whether we
%   have achieved the minimum sensing requirements for all the targets.
% =========================================================================
function [score, solutionFound, G] = solution_check(G, branchIDs, target, minEff)
solutionFound = false;

leafID = branchIDs(end);

% Check to see if the recently added branch results in a successful path to
% the goal.
[candPathIDs, candDist, candEff] = get_shortest_path(G, leafID);
rootID = candPathIDs(1);
candPath = G.V(candPathIDs);

t = candPath(end,:);
F = trapz(t, candEff);
Fmin = (target.tspan(2) - target.tspan(1)) * minEff;
score = F/Fmin;

if (score >= 1)
	% Update search tree with the best distance
	if ( candDist < G.BestPathDist )
        fprintf(1,'Old: %f, New: %f', G.BestPathDist, candDist);
        G.BestPathDist = candDist;
        G.Solution = candPathIDs;
       	solutionFound = true;
    end
end