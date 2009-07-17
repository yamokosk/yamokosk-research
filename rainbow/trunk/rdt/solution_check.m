% =========================================================================
% solution_check(G, branchIDs, numTargets, minEff)
%
%   Checks the newly created branch to see if we have achieved the sensing
%   requirements. Again the requirement for success is simply whether we
%   have achieved the minimum sensing requirements for all the targets.
% =========================================================================
function [rootID, leafID, score] = solution_check(G, branchIDs, target, minEff)

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