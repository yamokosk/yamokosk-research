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
function [Xsel, Wsel, IDsel] = select(G,n)

global tspan;

allNodes = G.V;
allNodeWeights = G.Wv;
allNodeDist = G.ShortestPathDistances;
numNodes = size(allNodes,2);

% Only interested in selecting from subset of nodes with path distances
% less that the current best solution path distance as well as nodes that
% have a timestamp less than the trajectory we are planning for.
lowDistNodeIDs = find( (allNodeDist < G.BestPathDist) );
nonEndpointNodeIds = find( allNodes(end,:)' < tspan(2) );
%validNodeIDs = find( (allNodeDist < G.BestPathDist) & (allNodes(end,:) < tspan(2)) );
validNodeIDs = intersect(lowDistNodeIDs, nonEndpointNodeIds);
validNodes = allNodes(:,validNodeIDs);
validNodeWeights = allNodeWeights(:,validNodeIDs);

% debug 
%fprintf(1,'Ignoring %5.2f percent of nodes.\n', (numNodes - length(validNodeIDs))/numNodes);

% Probabilistically select the most fit node to expand
%[Xsel,Wsel,IDtemp] = selsus(validNodes,validNodeWeights,3);
[Xsel,Wsel,IDtemp] = universal_selection(validNodes,validNodeWeights,n,'LinearRank',1e-1);

% Get back original IDs
IDsel = validNodeIDs(IDtemp);

% % Take best two or three
% [junk, ind] = sort(Wsel,2,'descend');
% 
% Xsel = Xsel(:,ind(1:2));
% Wsel = Wsel(ind(1:2));
% IDsel = IDsel(ind(1:2));
% 