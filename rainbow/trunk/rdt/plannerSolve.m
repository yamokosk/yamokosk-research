function Prob = plannerSolve(Prob)
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
Prob = initialize(Prob);

% Get current path length
best_path_length = Prob.solution.bestPathLength;

% Now just do the specified number of iterations
for iter = 1:num_iterations
    Prob = plannerIterate(best_path_length, Prob);
    
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
function Prob = initialize(Prob)

% Create a new solution data structure, if necessary
if isempty(Prob.solution) 
    G = graph();
    W0 = Prob.func_handles.neval(Prob.x0, Prob.targets, Prob.userdata);  % Evaluate the new node
    G = add_node(G, Prob.x0, W0);

    Prob.solution = struct('connectivityGraph', G, ...
                           'bestPathLength', inf, ...
                           'paths', {});
end