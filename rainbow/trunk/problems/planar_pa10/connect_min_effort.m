function [T, U, X, exitflag, exitmsg] = connect_min_effort(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, N)

direction = 'increasing';
if (tspan(2) < tspan(1))
    direction = 'decreasing';
end

% Define bounds and guess
tmin = [tspan(1), tspan(1)];
tmax = [tspan(2), tspan(2)];
xmin = [x0, x_lb, xf];
xmax = [x0, x_ub, xf];

alpha = linspace(0,1,N);
xguess = [x0'; xf'];
uguess = zeros(2, length(u_lb));
% xguess = zeros(N, length(x0));
% uguess = zeros(N, length(u_lb));

% for n = 1:N
%     xguess(n,:) = (x0 + alpha(n) * (xf - x0))';
% end

% Create the structs required by GPOCS
phases = struct( 'nodes',       N, ...
                 'time',        struct('min', tmin, 'max', tmax), ...
                 'states',      struct('min', xmin, 'max', xmax), ...
                 'controls',    struct('min', u_lb, 'max', u_ub) );

guess = struct( 'time',     [tspan(1); tspan(2)], ...
                'states',	[x0'; xf'], ...
                'controls',	uguess );

problem = struct( 'FUNCS',                  struct('ode', func2str(@planar_robots_ode), 'cost', func2str(@planar_cost)), ...
                  'name',                   'CONNECT_MIN_EFFORT', ...
                  'independent_variable',   direction, ...
                  'autoscale',              'on', ...
                  'phases',                 phases, ...
                  'guess',                  guess);

try
%     Must call GPOCS in a try-catch right now because sometimes GPOCS
%     crashes with the following error.. which I don't yet understand
%     fully:
%     
%       something about cross-over state identified.
    problem = gpocs('snopt7',problem,0);
catch
    err = lasterror;
    T = []; U = []; X = [];
    exitflag = 999;
    exitmsg = err.message;
    return;
end
solution = problem.solution;

% Parse snopt summary file (snoptpri.txt) and find the exit message.
% Ideally lookind for the following:
%   SNOPTB EXIT   0 -- finished successfully
fid = fopen('snoptpri.txt');
line = fgetl(fid);
while ( ~feof(fid) )
    ind = strfind(line, 'SNOPTB EXIT');
    if ( ~isempty(ind) )
        fclose(fid);
        ind = strfind(line, '0 -- finished successfully');
        line
        if ( ~isempty(ind) )
            exitflag = 0;
            exitmsg = 'finished successfully';
            T = solution.time;
            U = solution.controls;
            X = solution.states;
        else
            exitflag = 1;
            exitmsg = [line '\n'];
            T = [];
            U = [];
            X = [];
        end
        
        !del snopt*.txt
        return;
    end
    line = fgetl(fid);
end
error('The SNOPT algorithm did write results to a file that I could read!');