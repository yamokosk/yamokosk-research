function Result = rdt(Prob)

G_new = Prob.G;
iter = Prob.iter;

% General RDT methods
for i = 1:iter
    % Node selection
    [Ne,Ne_ind,Nr,G] = node_selection(G_new, Prob);
    
    % Node expansion
    %   Ni - intermediate nodes on path from Ne to Nr
    [Ni,We] = Prob.local_planner(Ne, Nr, Prob);
    
    % Evaluation
    G_new = node_evaluation(Ne_id, Ni, We, G_new);
end


% A node from the existing tree is selected as a location to add a
% branch. Selection of a particular node is usually based on
% probabilistic criteria that may require use of a valid distance
% metric
function [Ne,Ne_id,Nr,G] = node_selection(G, Prob)
Nr = Prob.node_generator(Prob);     % Generate random node
Wr = Prob.node_evaluate(Nr, Prob);  % Evaluate the new node
Ne_id = Prob.mfun(Nr, G.V, G.Wv);   % Select an existing node from the tree based
                                    % on this randomly generated node
G = add_node(G, Nr, Wr);            % Add node
Ne = G.V(Ne_id);                    % Get copy of existing node


% The new branch is evaluated according to performance criteria and
% often for connection to the goal configuration. Additionally, the new
% branch may be subdivided into multiple segments, thus adding several
% new nodes to the existing tree.
function G_new = node_evaluation(Ne_id, Ni, We, G_new)
ni = size(Ni,2); % number of intermediate nodes

if (ni > 0)
    % Evaluate the new nodes
    Wn = Prob.neval(Ni, Prob);
    % Add new nodes to tree
    [G_new,ind] = add_node(G_new, Ni, Wn);
    % Connect ne to ni(1)
    G_new = add_edge(G_new, Ne_id, ind(1), We(1));
    
    for i = 2:ni
        G_new = add_edge(G_new, ind(i-1), ind(i), Wi(i));
    end 
end
    


