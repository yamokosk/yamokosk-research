function [stats, branchLengths, branchSensingEffectiveness, branchSensingEfficiencies, nodesExtendedCount, h]  = compute_search_tree_stats(G,makeFigures)

% Want to make a histogram plot of something
leafAndRootIds = get_leaf_ids(G);
rootIds = get_root_ids(G);

leafIds = setdiff(leafAndRootIds, rootIds);
numLeafNodes = length(leafIds);

branchLengths = zeros(1,numLeafNodes);
branchSensingEffectiveness = zeros(1,numLeafNodes);

for n = 1:numLeafNodes
    [path, branchLengths(n), pathEff] = get_shortest_path(G, leafIds(n));
    nodesOnPath = G.NodeData(:,path);
    t = nodesOnPath(end,:);
    branchSensingEffectiveness(n) = trapz(t,pathEff);
end

branchSensingEfficiencies = branchSensingEffectiveness ./ branchLengths;

% nodes_extended = G.NodeExtendedCount;
% [ind,val] = find( G.NodeExtendedCount = 
% G.NodeExtendedCount;
% G.Seff;
% 
% N = hist(Y,X);


allNodeIds = 1:G.NodeCount;
allButLeafIds = setdiff(allNodeIds,leafIds);
nodesExtendedCount = full(G.NodeExtendedCount(1,allButLeafIds));

stats = struct('AvgBranchLength', mean(branchLengths), ...
               'StdBranchLength', std(branchLengths), ...
               'AvgBranchEfficiency', mean(branchSensingEfficiencies), ...
               'StdBranchEfficiency', std(branchSensingEfficiencies), ...
               'AvgNodeExtensions', mean(nodesExtendedCount), ...
               'StdNodeExtensions', std(nodesExtendedCount), ...
               'LeafToTotalNodeRatio', numLeafNodes/G.NodeCount);

h = [];
if (makeFigures)
    h = figure();
    set(h, 'Color', [1 1 1]);
    subplot(3,1,1);
    hist(branchLengths);
    xlabel('Branch lengths');
    ylabel('# of leaf nodes');
    subplot(3,1,2);
    hist(branchSensingEfficiencies);
    xlabel('Branch effectiveness');
    ylabel('# of leaf nodes');
    subplot(3,1,3);
    X = min(nodesExtendedCount):1:max(nodesExtendedCount);
    hist(nodesExtendedCount,X);
    xlabel('Number of Extensions');
    ylabel('Number of Nodes');
end
               

