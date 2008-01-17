function new_tree = rdt(tree,opts)
% tree.edge_graph = sparse weight matrix
% tree.node_list = list of nodes
% tree.node_weights = list of node weights
%tree = sparse(numNodes,numNodes);

% General RDT methods
for i = 1:opts.iter
    % Node selection
    node = node_selection(tree, opts);
    
    % Node expansion
    branch = node_expansion(node, tree, opts);
    
    % Evaluation
    tree = node_evaluation(branch, opts);
end

new_tree = [];


% A node from the existing tree is selected as a location to add a
% branch. Selection of a particular node is usually based on
% probabilistic criteria that may require use of a valid distance
% metric
function node = node_selection(tree,opts)
rand_node = opts.node_generator();
node = opts.selection_metric(rand_node,tree);

% A local planning method is used to extend a feasible trajectory from
% the selected node. The local goal for this trajectory branch is
% determined probabilistically.
function branch = node_expansion(node, tree, opts)
branch = [];


% The new branch is evaluated according to performance criteria and
% often for connection to the goal configuration. Additionally, the new
% branch may be subdivided into multiple segments, thus adding several
% new nodes to the existing tree.
function tree = node_evaluation(branch, opts)
tree = [];


