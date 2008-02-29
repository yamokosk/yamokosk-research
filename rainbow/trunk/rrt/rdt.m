function Prob = rdt(Prob)
G_new = [];
if isempty(Prob.G) 
    G_new = graph();
    W0 = Prob.node_evaluate(Prob.x0, Prob);  % Evaluate the new node
    G_new = add_node(G_new, Prob.x0, W0);
else
    G_new = Prob.G; 
end

iter = Prob.iter;
fprintf('Iteration     Status\n');
fprintf('---------     ------\n');

% General RDT methods
for i = 1:iter
    % Node selection
    [Ne,Ne_id,Nr,G_new] = node_selection(G_new, Prob);
    
    % Node expansion
    %   Ni - intermediate nodes on path from Ne to Nr
    [Ni,We] = Prob.local_planner(Ne, Nr, Prob);
    
    % Evaluation
    G_new = node_evaluation(Ne_id, Ni, We, G_new, Prob);
    
    % Print info about this iteration
    if (isempty(Ni))
        fprintf('  %d           Failed\n', i);
    else
        fprintf('  %d           Completed\n', i);
        gcf; hold on;
        N = size(Ni,2);
        Xsrc = zeros(2,N);
        Xsen = zeros(2,N);
        for n = 1:N
            qsrc = ( Prob.x_range(2:4).*Ni(2:4,n) + Prob.x_ub(2:4) + Prob.x_lb(2:4) ) ./ 2;
            Tsrc = fkine_planar_pa10(qsrc,Prob.userdata.r1);
            plot_planar_pa10(qsrc, Prob.userdata.r1, gcf);
            Xsrc(:,n) = Tsrc(1:2,4);
            
            qsen = ( Prob.x_range(8:10).*Ni(8:10,n) + Prob.x_ub(8:10) + Prob.x_lb(8:10) ) ./ 2;
            Tsen = fkine_planar_pa10(qsen,Prob.userdata.r2);
            plot_planar_pa10(qsen, Prob.userdata.r2, gcf);
            Xsen(:,n) = Tsen(1:2,4);
            drawnow;
        end
        a = 2;
        %plot(Xsrc(1,:), Xsrc(2,:), 'k.-');
        %plot(Xsen(1,:), Xsen(2,:), 'r.-');
    end
    
    
end

Prob.G = G_new;


% A node from the existing tree is selected as a location to add a
% branch. Selection of a particular node is usually based on
% probabilistic criteria that may require use of a valid distance
% metric
function [Ne,Ne_id,Nr,G] = node_selection(G, Prob)

for n = 1:100
    Nr = Prob.node_generator(Prob);     % Generate random node
    Wr = Prob.node_evaluate(Nr, Prob);  % Evaluate the new node
    Ne_id = Prob.node_select(Nr, G.V, G.Wv, Prob);   % Select an existing node from the tree based
                                                     % on this randomly generated node
    if (Ne_id > 0)
        G = add_node(G, Nr, Wr);            % Add node
        Ne = G.V(Ne_id);                    % Get copy of existing node
        break;
    end
end


% The new branch is evaluated according to performance criteria and
% often for connection to the goal configuration. Additionally, the new
% branch may be subdivided into multiple segments, thus adding several
% new nodes to the existing tree.
function G_new = node_evaluation(Ne_id, Ni, We, G_new, Prob)
ni = size(Ni,2); % number of intermediate nodes

if (ni > 0)
    Ni = Ni(:,2:end-1);
    We = We(2:end-1);
    ni = size(Ni,2);
    
    % Evaluate the new nodes
    Wn = Prob.node_evaluate(Ni, Prob);
    % Add new nodes to tree
    [G_new,ind] = add_node(G_new, Ni, Wn');
    % Connect ne to ni(1)
    G_new = add_edge(G_new, Ne_id, ind(1), We(1));
    
    for i = 2:ni
        G_new = add_edge(G_new, ind(i-1), ind(i), We(i));
    end 
end
    


