function [G,ds] = constructPRM(S,opts)

% Check opts for required functions
if ~isfield(opts,'LocalPlanner') 
    error('Local planning method not specified in options struct.');
end
lpm = opts.LocalPlanner;
if ~isa(lpm, 'function_handle')
    error('opts.LocalPlanner must be a function handle');
end

% Check opts for required functions
if ~isfield(opts,'Validate') 
    error('Validation method not specified in options struct.');
end
validate = opts.Validate;
if ~isa(validate, 'function_handle')
    error('opts.Validate must be a function handle');
end   

% Random samples
numSamples = size(S,2);

% Setup kd-tree stuff
numNhbrs = 5;
if isfield(opts,'NumNeighbors') numNhbrs = opts.NumNeighbors; end
initKDTree(S);

% Initialize graph and disjoint set
G = sparse(numSamples,numSamples);
ds = disjointset(numSamples);

h1 = figure(1);
set(h1,'Color',[1 1 1],'Position',[100, 200, 500, 500]);
spy(G)
title('Adjacency graph');
h2 = figure(2);
set(h2,'Color',[1 1 1],'Position',[600, 200, 500, 500]);
spy(ds)
title('Connected components');
drawnow;

% PRM construction algorithm
startTime = now;
indices = randperm(numSamples);
for kk = 1:numSamples
    ii = indices(kk);
    c = S(:,ii);
    [nbhrs,dist,ind] = searchKDTree(c,numNhbrs);
    
    for n = 1:length(ind)
        if ~same_connected_component(ds,ii,ind(n))
            if lpm(c,nbhrs(:,n),opts)
                G(ii,ind(n)) = dist(n); % Record new edge
                %G(ind(n),ii) = dist(n);
                ds = union_set(ds,ii,ind(n)); % Update connect components
            end
        end
    end
    
    if (mod(kk,100) == 0) 
        elapsedTime = now - startTime;
        remainingTime = (elapsedTime/kk)*(numSamples-kk);
        expectedComplete = now + remainingTime;
        fprintf(1,'Completed %d of %d samples.  Remaining: %s, Expected completion: %s\n',kk,numSamples,datestr(remainingTime,13),datestr(expectedComplete,21)); 
        figure(1);
        spy(G);
        title('Adjacency graph');
        figure(2);
        spy(ds);
        title('Connected components');
        drawnow;
    end
end

savefile = ['PRMComplete-' datestr(now,30) '.mat'];
save(savefile, 'G', 'ds'); 
resetKDTree;