% =========================================================================
% select(G, targets, minEff)
%
%   Probabilistically selects the next target to sense. Basic operation:
%       1. Probabilistically select the next node to expand based on the
%       node weights. Currently uses the stochastic universal sampling 
%       algorithm from the GA communitiy. Reference:
%            Baker, J.E., "Reducing bias and inefficiency in the selection 
%               algorithm", Proceedings of the Second International 
%               Conference on Genetic Algorithms and their application,
%               pp.14-21, 1987.
%       2. Then a target is selected which must meet the following
%       criteria:
%           I. Has a time greater than the selected node.
%               AND
%           IIa. Has a minimum effectiveness less than the required
%           minimum.
%               OR
%           IIb. Has the smallest effectiveness of all furutre targets.
% =========================================================================
function [Xsel, Wsel, IDsel] = select(G)

allNodes = G.V;
allNodeWeights = G.Wv;

% Probabilistically select the most fit node to expand
[Xsel,Wsel,IDsel] = selsus(allNodes,allNodeWeights,3);

% % Take best two or three
% [junk, ind] = sort(Wsel,2,'descend');
% 
% Xsel = Xsel(:,ind(1:2));
% Wsel = Wsel(ind(1:2));
% IDsel = IDsel(ind(1:2));
% 