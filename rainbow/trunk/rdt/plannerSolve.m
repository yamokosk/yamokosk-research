function Prob = plannerSolve(Prob, target)
% plannerSolve - Solves the vantage point planning problem for my thesis!
%
%   Prob = plannerSolve(Prob) attempts to solve the vantage point planning
%   problem for a problem domain specified in Prob. See plannerAssign for 
%   information on constructing a proper Problem data structure.
%
%   Assumptions
%   *   Continuous time dynamic systems.   

% Create some local variables for ease of readability
num_iterations = Prob.iterations;
outfun = Prob.func_handles.outfun;
goalfun = Prob.func_handles.goalfun;

% Do some initialization stuff
outfun('init', '', Prob);
Prob = initialize(Prob, target);

% Get current path length and graph
best_path_length = Prob.solution.bestPathLength;
G = Prob.solution.connectivityGraph;
Seff = Prob.solution.sensing_effectiveness;

% Now just do the specified number of iterations
for iter = 1:num_iterations   
    % Find either an unsensed target or target with min sensing
    % effectiveness
    Ti = findMinTarget(targets, Seff);
    
    % Deal with this target
    Prob = plannerIterate(Prob, Ti, best_path_length);

    % Compute shortest path from x0 to all other nodes.
    [d pred] = shortest_paths(g.E,1,'edge_weight',edge_weight_vector(G.E,G.Wv));
    
    % Check to see if any of the paths result is meeting our goal.
    ind = find( d < best_path_length );
    for npath = length(ind):-1:2
        path_candidate = path_from_pred(pred,ind(npath));
        
        % Compute sensing effectiveness
        Se = computeSensingEffectiveness(G.V(path_candidate), targets);
    end
        
        
    if ( goalfun(Prob) )
        completed_paths = current_path;
        if (path_length < best_path_length)
            best_path_length = path_length;
        end 
    else
        outfun('iter', 'Complete', Prob);
    end
end

% Notify output function we are done
outfun('done', '', Prob);


% =========================================================================
% initialize(Prob)
%
%   Performs some simple initialization steps for the planner, including:
%       * Initialize output function
%       * Initialize solution data structure, if needed
% =========================================================================
function Prob = initialize(Prob, target)

% Create a new solution data structure, if necessary
if isempty(Prob.solution) 
    G = graph();
    W0 = Prob.func_handles.neval(Prob.x0, Prob.targets, Prob.userdata);  % Evaluate the new node
    G = add_node(G, Prob.x0);
    
    Seff = zeros(1,target.N);
    Prob.solution = struct('connectivityGraph', G, ...
                           'bestPathLength', inf, ...
                           'sensing_effectiveness', Seff);
end


% =========================================================================
% findMinTarget(targets, Seff)
% =========================================================================
function Ti = findMinTarget(targets, Seff)

ind = find( Seff == min(Seff) );

Ti = targets.data(:,ind(1));