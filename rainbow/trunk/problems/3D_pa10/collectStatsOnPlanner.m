function [G, meanWeight, maxWeight, stdWeight, maxScore, distOfBest] = collectStatsOnPlanner(N,X0,target,f,opts,udata)

% reset random number generators
meanWeight = zeros(1,N);
maxWeight = zeros(1,N);
stdWeight = zeros(1,N);
maxScore = zeros(1,N);
distOfBest = zeros(1,N);
rand('state',sum(100*clock));
randn('state',sum(113*clock));
for n = 1:N
    G = plannerSolve(X0, target, f, opts, udata);
    opts.PriorSearchTree = G;

    meanWeight(n) = mean(G.Wv);
    maxWeight(n) = max(G.Wv);
    stdWeight(n) = std(G.Wv);
    
    %bestLeafID = find( G.Wv == 
    %maxScore(n) = G.BestPathScore;
    
    [path, dist, eff] = get_shortest_path(G, G.BestLeafID);
    distOfBest(n) = dist;
end