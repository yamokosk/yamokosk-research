function Prob = plannerAssignFromDir(directory)

% Read problem def file
def = readProblemDef( fullfile(directory, 'problem.def') );

% Set path
path(path, directory);

% Set problem structure
f = struct( 'ngen', def.node_generator, ...
            'nsel', def.node_selector, ...
            'neval', def.node_evaluator, ...
            'lp', def.local_planner, ...
            'goalfun', def.goal_function);
udata = struct( 'odefun', def.ode_function );

if ( isfield(def, 'output_function') )
    Prob = plannerAssignFromArgs(def.x0, def.x_lb, def.x_ub, def.u_lb, def.u_ub, def.iter, f, def.name, udata, def.output_function );
else
    Prob = plannerAssignFromArgs(def.x0, def.x_lb, def.x_ub, def.u_lb, def.u_ub, def.iter, f, def.name, udata );
end