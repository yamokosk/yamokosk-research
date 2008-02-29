function [t, U_opt, X_opt] = connect_min_effort(t0, x0, tf, xf, N, odefun, costfun, udata)

problem.FUNCS.ode   = func2str(odefun);
problem.FUNCS.cost  = func2str(costfun);
problem.name        = 'CONNECT';
if (tf > t0)
    problem.independent_variable = 'increasing';
else
    problem.independent_variable = 'decreasing';
end
% Can only use automatic derivatives if not using mex files for dynamics or
% cost function.
%problem.Derivatives = 'automatic'; 
%problem.autoscale   = 'on';
problem.autoscale   = 'on';

%rmin = [udata.r1.qmin'; -udata.r1.qpmax'; udata.r2.qmin'; -udata.r2.qpmax'];
%rmax = [udata.r1.qmax'; udata.r1.qpmax'; udata.r2.qmax'; udata.r2.qpmax'];

%umin = [-udata.r1.umax'; -udata.r2.umax'];
%umax = [udata.r1.umax'; udata.r2.umax'];

phases(1).nodes = N;
phases(1).time.min        = [t0 t0];
phases(1).time.max        = [tf tf];
phases(1).states.min      = [x0 -1*ones(size(x0)) xf]; % Probably want to put lower and upper bounds in here
phases(1).states.max      = [x0 ones(size(x0)) xf];
phases(1).controls.min    = -1*ones(6,1);
phases(1).controls.max    = ones(6,1);

guess(1).time          = [t0; tf];
guess(1).states        = [x0'; xf'];
guess(1).controls      = zeros(2,6);

problem.phases   = phases;
problem.guess    = guess;
problem          = gpocs('snopt7',problem,0);
solution         = problem.solution;

t = solution.time;
U_opt = solution.controls;
X_opt = solution.states;