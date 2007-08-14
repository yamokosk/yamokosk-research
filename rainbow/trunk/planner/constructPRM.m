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
xy = [S(1,:)', S(2,:)'];
plot(S(1,:),S(2,:),'k.');
hold on;
pause;

% Setup kd-tree stuff
numNhbrs = 5;
if isfield(opts,'NumNeighbors') numNhbrs = opts.NumNeighbors; end
initKDTree(S);

% Initialize graph and disjoint set
G = sparse(numSamples,numSamples);
ds = disjointset(numSamples);

% PRM construction algorithm
for ii = 1:numSamples
    c = S(:,ii);
    [nbhrs,dist,ind] = searchKDTree(c,numNhbrs);
    
    for n = 1:length(ind)
        if ~same_connected_component(ds,ii,ind(n))
            if lpm(c,nbhrs(:,n),opts)
                G(ii,ind(n)) = dist(n); % Record new edge
                ds = union_set(ds,ii,ind(n)); % Update connect components
            end
        end
    end
    
    if (mod(ii,10) == 0) disp(['Iteration: ' num2str(ii)]); gplot(G,xy); drawnow; end
end

resetKDTree;