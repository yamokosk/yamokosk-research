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
    % Node selection
    [Neighbors, ID, QueryStates, G_new] = node_selection(G_new, Prob);
    [nstates, nconnects] = size(QueryStates);
    
    % Node expansion
    for j = 1:nconnects
        [Path, EdgeWeights, ExitFlag, ExitMsg] = Prob.local_planner(Neighbors(:,j), QueryStates(:,j), Prob);
    
        status = 'Failed';
        if ((ExitFlag == 0) && (size(Path,2) > 1))
            % Evaluation
            status = 'Complete';
            G_new = node_evaluation(ID(j), Path, EdgeWeights, G_new, Prob);
        end
        
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
function G_new = node_evaluation(ID, Path, EdgeWeights, G_new, Prob)

% The first node should already be in the tree. So first evaluate and add
% the new nodes
NewNodes = Path(:,2:end);
NewNodeWeights = Prob.node_evaluate(NewNodes, Prob);
[G_new,NewNodeIDs] = add_node(G_new, NewNodes, NewNodeWeights);

% Next make an edge between existing node in the tree and our first new
% node.
G_new = add_edge(G_new, ID, NewNodeIDs(1), EdgeWeights(1));

% Then use a for loop to add the rest of the edges
nedges = length(NewNodeIDs);
for i = 2:nedges
    G_new = add_edge(G_new, NewNodeIDs(i-1), NewNodeIDs(i), EdgeWeights(i));
end 