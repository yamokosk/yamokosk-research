function fitness = neval(state, targets, udata)

ntargets = size(targets, 2);
fitness = 0;

for n = 1:ntargets
    dist_fitness = distance_metric(state, targets(:,n));
    fitness = max(dist_fitness, fitness);
end