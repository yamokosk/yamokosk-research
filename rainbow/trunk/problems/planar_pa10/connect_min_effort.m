function [T, U, X, exitflag, exitmsg] = connect_min_effort(tspan, x0, xf, x_lb, x_ub, u_lb, u_ub, N, udata)

direction = 'increasing';
if (tspan(2) < tspan(1))
    direction = 'decreasing';
end

% See if I can do a better job generating an initial guess
dt = tspan(2) - tspan(1);
ind = [2, 3, 5];

qpp_src = (xf(4:6,1) - x0(4:6,1))/dt;
tau_src_0 = torque(udata.rsrc, x0(1:3,1), x0(4:6,1), qpp_src);
tau_src_f = torque(udata.rsrc, xf(1:3,1), xf(4:6,1), qpp_src);

qpp_sen = (xf(10:12,1) - x0(10:12,1))/dt;
tau_sen_0 = torque(udata.rsen, x0(7:9,1), x0(10:12,1), qpp_sen);
tau_sen_f = torque(udata.rsen, xf(7:9,1), xf(10:12,1), qpp_sen);

xguess = [x0'; xf'];
uguess = [tau_src_0(ind)', tau_sen_0(ind)'; ...
          tau_src_f(ind)', tau_sen_f(ind)'];

% Compute 
% Define bounds and guess
tmin = [tspan(1), tspan(1)];
tmax = [tspan(1), tspan(2)];
xmin = [x0, x_lb, xf];
xmax = [x0, x_ub, xf];

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