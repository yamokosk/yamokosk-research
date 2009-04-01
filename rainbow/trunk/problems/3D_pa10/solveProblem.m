clear;
close all;

global tspan;

% load robots
conffile = 'pa10_dh.conf';
[src,sen] = loadRobots( conffile );

% load traj
[t,pp] = generate_traj_from_knee_motion();
var = ([0.05, 0.05, 0.05, 10*(pi/180), 10*(pi/180), 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]').^2;
target = struct('pp', pp, 'variance', var, 'tspan', [0,t(end)]);
tspan = [0, t(end)];

% create structures
f = struct('lp',@constrained_straight_line_planner, ...
            'ngen',@ngen, ...
            'neval',@neval);

params = struct('phi_std',  (pi/4)/6, ...   % Max allowable deviation in angle between source-target vector and desired view vector
                'r_min',    0.6, ...        % Min distance between source and target
                'r_max',    1, ...          % Max distance between source and target
                'D_ref',    0.09, ...       % Approximate diameter of reference object in meters
                'S_width',  0.179, ...      % Width of Varian X-ray detector
                'Rmin',     0.30, ...       % Min percentage reference object occupies in image
                'Rmax',     0.80, ...       % Max percentage reference object occupies in image
                'theta_std',0.0698/6);      % Max allowable deviation between actual view vector and source-target vector

udata = struct('rsrc', src, ...
               'rsen', sen, ...
               'params', params);


% Get and mess with options
opts = plannerSolve('defaults');
opts.Display = 'iter';
opts.MaxIter = 5;
opts.TimeStep = 1/25;
opts.SkewFactor = 0.5;

% generate initial guess           
X0 = [];
c=0;
tgen = rand(100,1) * 0.25 * max(t);
for n = 1:100
    x = ppval(pp,tgen(n));
    Vtemp = f.ngen(0, x, udata);
    if ( ~isempty(Vtemp) )
        X0 = [X0, Vtemp];
        c = c+1;
    end
end
fprintf(1,'Seeding with %d states\n',c);

% Setup plot
% plotResults([],[],true);

% Find a solution!
% N = 100; M = 3;
% meanWeight = zeros(M,N);
% stdWeight = zeros(M,N);
% maxWeight = zeros(M,N);
% maxScore = zeros(M,N);
% distOfBest = zeros(M,N);
% rand('state',sum(100*clock));
% randn('state',sum(113*clock));
% plotResults([],[],true);
% for m = 1:M
%     [G, meanWeight(m,:), maxWeight(m,:), stdWeight(m,:), maxScore(m,:), distOfBest(m,:)] = ...
%          collectStatsOnPlanner(N,X0,target,f,opts,udata);
%     plotResults(G,N,false);
% end

% % Lets collect some stats on this algorithm!
% M = 2;
% N = 50;
% meanWeight = zeros(M,N);
% stdWeight = zeros(M,N);
% maxWeight = zeros(M,N);
% maxScore = zeros(M,N);
% distOfBest = zeros(M,N);
% for m = 1:M
%     [meanWeight(m,:), maxWeight(m,:), stdWeight(m,:), maxScore(m,:), distOfBest(m,:)] = ...
%         collectStatsOnPlanner(N,X0,target,f,opts,udata);
% end
% 
% figure(1)
% plot((1:N)*5, mean(meanWeight), 'k:', (1:N)*5, mean(maxWeight), 'k-', (1:N)*5, mean(stdWeight), 'k-.')
% title('Node weight statistics')
% xlabel('Iterations')
% ylabel('Node weight')
% legend('Mean', 'Max', 'Std')
% set(1,'Color',[1 1 1]);
% 
% figure(2)
% plot((1:N)*5, mean(maxScore), 'k-')
% title('Max path efficiency')
% xlabel('Iterations')
% ylabel('Path sensing efficiency')
% set(2,'Color',[1 1 1])
% 
% figure(3)
% plot((1:N)*5, mean(distOfBest), 'k-')
% title('Dist of best leaf node')
% xlabel('Iterations')
% ylabel('Distance')
% set(3,'Color',[1 1 1])


% diary off;
% save 'testrun.mat';