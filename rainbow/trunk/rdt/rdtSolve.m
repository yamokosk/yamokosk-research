function Prob = rdtSolve(Prob)

% Initialize output function
Prob.output_fcn(0, [], [], [], 'init', Prob);

G_new = [];
if isempty(Prob.G) 
    G_new = graph();
    W0 = Prob.node_evaluate(Prob.x0, Prob);  % Evaluate the new node
    G_new = add_node(G_new, Prob.x0, W0);
else
    G_new = Prob.G; 
end

% The RDT loop
iter = Prob.iter;
for i = 1:iter
    % Branch selection not node selection.. need to change this.
    [Neighbors, ID, QueryStates, G_new] = node_selection(G_new, Prob);
    [nstates, nconnects] = size(QueryStates);
    
    % Node expansion
    for j = 1:nconnects
        % branch.root = node_id
        % branch.terminal = node_id
        % branch.path = [node_ids]
        [branch,exitflag,exitmsg] = Prob.local_planner(Neighbors(:,j), QueryStates(:,j), Prob);
    
        % Evaluate new branch
        
        % Collision check is delayed now. So need to add infrastucture to
        % load SceneML file and check collision of best path. In fact the
        % way to do this is check the new branches score and determine if
        % the addition of this new branch results in a better path. If so
        % its going to now be our current best solution. Before we accept
        % it as the better solution, perform the delayed collision
        % detection.
        status = 'Failed';
        if ((branch.ExitFlag == 0) && (size(branch.Path,2) > 1))
            % Evaluation
            status = 'Complete';
            q_branch = branch_evaluation(branch);
        end
        
        % If q_branch > q_current, replace
        % Iteration output
        if ( Prob.output_fcn('Connect', status, Path, ExitMsg, 'connect', Prob) ) break; end
    end
    
    % Iteration output
    if ( Prob.output_fcn('Iteration', 'Complete', [], [], 'iter', Prob) ) break; end
end

Prob.G = G_new;

% Notify output function we are done
Prob.output_fcn(0, [], [], [], 'done', Prob);


% A node from the existing tree is selected as a location to add a
% branch. Selection of a particular node is usually based on
% probabilistic criteria that may require use of a valid distance
% metric
function [Neighbors,ID,QueryStates,G] = node_selection(G, Prob)

for ii = 1:5
    % Generate random nodes
    Xr = Prob.node_generator(Prob);
    
    % Get the nearest neighbors to the randomly generated states from
    % the existing tree.
    ID = Prob.node_select(Xr, G.V, G.Wv, Prob);
    ind = find(ID > 0);
    
    if ( ~isempty(ind) )
        ID = ID(ind);
        Neighbors = G.V(ID);
        QueryStates = Xr(:,ind);
        return;
    end
end
error('Gave up selecting a new node for expansion after five attempts.');


% The new branch is evaluated according to performance criteria and
% often for connection to the goal configuration. Additionally, the new
% branch may be subdivided into multiple segments, thus adding several
% new nodes to the existing tree.
function q_branch = branch_evaluation(branch)

% Compute impact this branch has on all targets
for i = 1:num_targets
    q_branch(i) = effectiveness_metric(target(i),branch);
end





% The first node should already be in the tree. So first evaluate and add
% the new nodes
NewNodes = Path(:,2:end);
NewNodeWeights = Prob.node_evaluate(NewNodes, Prob);
[G_new,NewNodeIDs] = add_node(G_new, NewNodes, NewNodeWeights);

% Next make an edge between existing node in the tree and our first new
% node.
%G_new = add_edge(G_new, ID, NewNodeIDs(1), EdgeWeights(1));
IDWeight = G_new.Wv(ID);
G_new = add_edge(G_new, ID, NewNodeIDs(1), (IDWeight + NewNodeWeights(1))/2 );

% Then use a for loop to add the rest of the edges
nedges = length(NewNodeIDs);
for i = 2:nedges
    %G_new = add_edge(G_new, NewNodeIDs(i-1), NewNodeIDs(i), EdgeWeights(i));
    G_new = add_edge(G_new, NewNodeIDs(i-1), NewNodeIDs(i), (NewNodeWeights(i) + NewNodeWeights(i-1))/2 );
end 