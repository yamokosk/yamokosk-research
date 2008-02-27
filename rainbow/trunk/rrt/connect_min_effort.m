function [t, U_opt,X_opt] = connect_min_effort(t0, x0, tf, xf, odefun, costfun, udata)

problem.FUNCS.ode   = func2str(odefun);
problem.FUNCS.cost  = func2str(costfun);
problem.name        = 'CONNECT';
problem.independent_variable = 'increasing';
%problem.Derivatives = 'automatic';
problem.autoscale   = 'on';

ns = length(x0);

rmin = [udata.r1.qmin'; udata.r1.qpmin'; udata.r2.qmin'; udata.r2.qpmin];
rmax = [udata.r1.qmax'; udata.r1.qpmax'; udata.r2.qmax'; udata.r2.qpmax];

phases(1).nodes = N;
phases(1).time.min        = [0 t0];
phases(1).time.max        = [0 tf];
phases(1).states.min      = [x0 -pi/2*ones(ns,1) xf]; % Probably want to put lower and upper bounds in here
phases(1).states.max      = [x0  pi*ones(ns,1) xf];
%phases(1).controls.min    = [-1; -1];
%phases(1).controls.max    = [ 1;  1];
phases(1).controls.min    = -5*ones(6,1);
phases(1).controls.max    = 5*ones(6,1);

guess(1).time          = [t0; tf];
guess(1).states        = [x0'; xf'];
% guess(1).states(:,1)   = [0; 0];
% guess(1).states(:,2)   = [0; 0];
% guess(1).states(:,3)   = [0.5; 0.5];
% guess(1).states(:,4)   = [.522; .522];
guess(1).controls      = zeros(2,6);
%guess(1).controls(:,1) = [1; -1];
%guess(1).controls(:,2) = [1; -1];

problem.phases   = phases;
problem.guess    = guess;
problem          = gpocs('snopt7',problem,0);
solution         = problem.solution;

t = solution.time;
U_opt = solution.controls;
X_opt = solution.states;