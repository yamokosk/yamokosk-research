function def = readProblemDef(filename)

fid = fopen(filename);

if ( fid < 0 )
    error(['Failed to open problem definition: ' filename]);
end
def = [];

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
    switch (token)
        case 'name'
            C = textscan(line,'%s %s');
            def.(token) = char(C{1,2});
        case {'x0','x_lb','x_ub','u_lb','u_ub'}
            len = length(token);
            C = textscan(line(len+1:end),'%f');
            def.(token) = C{1};
        case 'iterations'
            C = textscan(line, '%s %d');
            def.iter = C{1,2};
        case {'ode_function','node_generator','node_evaluator','node_selector','local_planner','output_function','goal_function'}
            C = textscan(line, '%s %s');
            if ( isempty( C{1,2} ) )
                def.(token) = 0;
            else
                def.(token) = str2func(char(C{1,2}));
            end
        case ''
            continue;
        otherwise
            error(['Unrecognized token: ' token]);
    end
end

fclose(fid);