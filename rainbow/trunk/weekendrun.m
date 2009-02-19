clear all;
clc;

% set path
probdir = fullfile( pwd, 'problems', 'planar_pa10' );
path(path, probdir);

% load robots
conffile = 'pa10_planar.conf';
[src,sen] = planar_load_robots( fullfile( probdir, conffile ) );

% load traj
load( fullfile( probdir, 'traj.mat' ) );
time = targets(end,:);
data = targets(1:end-1,:);
pp = interp1(time,data','spline','pp');
var = ([0.01; 0.01; 0.05; 0.05; pi/6]/6).^2;
target = struct('tspan',[time(1), time(end)], 'pp', pp, 'variance', var);

% read prob def
def = readProblemDef( fullfile(probdir, 'problem.def') );

% create structures
f = struct('lp',def.local_planner, ...
            'ngen',def.node_generator, ...
            'neval',def.node_evaluator);

udata = struct('rsrc', src, ...
               'rsen', sen, ...
               'x_lb', def.x_lb, ...
               'x_ub', def.x_ub, ...
               'u_lb', def.u_lb, ...
               'u_ub', def.u_ub, ...
               'UseGPOCS', false);

% generate initial guess           
N = 30; c = 1;
X0 = zeros(13,N*3);
rand('state',sum(100*clock));
for id = [1, 2, 3]
    for n = 1:N
        Vtemp = f.ngen(time(id), targets(:,id), udata);
        if ( ~isempty(Vtemp) )
            X0(:,c) = Vtemp;
            c = c+1;
        end
    end
end

% Set initial time to zero
X0(end,:) = 0;

% Get and mess with options
opts = plannerSolve('defaults');
opts.Display = 'iter';
opts.MaxIter = 100;
opts.TimeStep = 0.25;
opts.SkewFactor = 0.5;

% diary 'screencapture2.txt';
% tic;
% G = plannerSolve(X0, targets, f, opts, udata);
% toc
% diary off;
% save 'testrun.mat';