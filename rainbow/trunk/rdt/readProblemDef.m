function def = readProblemDef(filename)

fid = fopen(filename);

if ( fid < 0 )
    error(['Failed to open problem definition: ' filename]);
end
def = struct('name',    '', ...
             'x0',      [], ...
             'x_lb',    [], ...
             'x_ub',    [], ...
             'u_lb',    [], ...
             'u_ub',    [], ...
             'iterations',  0, ...
             'collision_check', false, ...
             'ode_function', 0, ...
             'node_generator', 0, ...
             'node_evaluator', 0, ...
             'node_selector', 0, ...
             'local_planner', 0, ...
             'goal_function', 0, ...
             'output_function', 0, ...
             'collision_test_function', 0);

while ( ~feof(fid) )
    line = fgetl(fid);
    
    % Ignore empty lines
    if ( isempty(line) )
        continue;
    end
    
    % Ignore comments
    if ( line(1) == '#' )
        continue;
    end
    
    token = sscanf(line, '%s', 1);
    if ( ~isfield(def, token) )
        warning(['Unrecognized keyword ''' token '''. Ignoring it.']);
        continue;
    end
    
    switch (token)
        case 'name'
            C = textscan(line,'%s %s');
            def.(token) = char(C{1,2});
        case {'x0','x_lb','x_ub','u_lb','u_ub'}
            len = length(token);
            C = textscan(line(len+1:end),'%f');
            def.(token) = C{1};
        case {'iterations','collision_check'}
            C = textscan(line, '%s %d');
            def.(token) = C{1,2};
        case 'collision_check'
            C = textscan(line, '%s %d');
            def.collision_check = C{1,2};
        case {'ode_function','node_generator','node_evaluator','node_selector','local_planner','output_function','collision_test_function'}
            C = textscan(line, '%s %s');
            def.(token) = str2func(char(C{1,2}));
        case ''
            continue;
        otherwise
            %error(['Unrecognized token: ' token]);
    end
end

fclose(fid);