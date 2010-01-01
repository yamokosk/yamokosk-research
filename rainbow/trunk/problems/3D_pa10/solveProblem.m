% clear;
% close all;
% 
% global tspan;
% 
% % load robots
% conffile = 'pa10_dh.conf';
% [src,sen] = loadRobots( conffile );
% 
% % load traj
% [t,pp] = generate_traj_from_knee_motion();
% var = ([0.01, 0.01, 0.01, 5*(pi/180), 5*(pi/180), 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]').^2;
% target = struct('pp', pp, 'variance', var, 'tspan', [0,t(end)]);
% tspan = [0, t(end)];
% 
% % create structures
% % f = struct('lp',@constrained_straight_line_planner, ...
% f = struct('lp',@straight_line_planner, ...
%             'ngen',@ngen, ...
%             'neval',@neval, ...
%             'collisionCheck', @ccheck);
% 
% params = struct('phi_std',  (pi/4)/6, ...   % Max allowable deviation in angle between source-target vector and desired view vector
%                 'r_min',    0.3, ...        % Min distance between source and target 0.6
%                 'r_max',    0.7, ...          % Max distance between source and target 0.7
%                 'D_ref',    0.09, ...       % Approximate diameter of reference object in meters
%                 'S_width',  0.179, ...      % Width of Varian X-ray detector
%                 'Rmin',     0.15, ...       % Min percentage reference object occupies in image 0.3
%                 'Rmax',     0.50, ...       % Max percentage reference object occupies in image 0.8
%                 'theta_std',0.0698/6);      % Max allowable deviation between actual view vector and source-target vector
% 
% udata = struct('rsrc', src, ...
%                'rsen', sen, ...
%                'params', params);
% 
% 
% % Get and mess with options
% opts = plannerSolve('defaults');
% opts.Display = 'iter';
% % opts.MaxIter = 5;
% % opts.TimeStep = 1/25;
% % opts.SkewFactor = 0.5;
% 
% opts.MaxIter = 50;
% opts.SkewFactor = 1;
% opts.MinTimeStep = 0.01;
% opts.MaxTimeStep = 0.1;
% opts.DoCollisionCheck = 1;
% opts.MinSenEff = 0.80;
% 
% % generate initial guess           
% X0 = [];
% c=0;
% % tgen = rand(100,1) * 0.5 * max(t);
% % for n = 1:10
% %     x = ppval(pp,tgen(n));
% %     Vtemp = f.ngen(0, x, udata);
% %     if ( ~isempty(Vtemp) )
% %         X0 = [X0, Vtemp];
% %         c = c+1;
% %     end
% % end
% tgen = 0.2;
% x = ppval(pp,tgen);
% best_score = -inf;
% for n = 1:200
%     Vtemp = f.ngen(0, x, udata);
%     if ( ~isempty(Vtemp) )
%         Vval = f.neval(Vtemp, x, udata);
%         incollision = ccheck(Vtemp,udata);
%         if ( ( Vval > best_score ) && ~incollision )
%             X0 = Vtemp;
%             best_score = Vval;
%             c = 1;
%             fprintf(1, 'Best score: %f\n', best_score);
%         end
%     end
% end
% fprintf(1,'Seeding with %d states\n',c);

% G = plannerSolve(X0, target, f, opts, udata);
% for n = 1:20
%     opts.PriorSearchTree = G;
%     G = plannerSolve(X0, target, f, opts, udata);
%     save recent_results G
% end

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

% Lets collect some stats on this algorithm!
M = 5;
N = 100;
opts.MaxIter = 5;
meanWeight = zeros(M,N);
stdWeight = zeros(M,N);
maxWeight = zeros(M,N);
maxScore = zeros(M,N);
distOfBest = zeros(M,N);
for m = 1:M
    [G, meanWeight(m,:), maxWeight(m,:), stdWeight(m,:), maxScore(m,:), distOfBest(m,:)] = ...
        collectStatsOnPlanner(N,X0,knee_target,f,opts,udata);
end

% figure(1)
% plot((1:N)*opts.MaxIter, mean(meanWeight), 'k:', (1:N)*opts.MaxIter, mean(maxWeight), 'k-', (1:N)*opts.MaxIter, mean(stdWeight), 'k-.')
% title('Node weight statistics')
% xlabel('Iterations')
% ylabel('Node weight')
% legend('Mean', 'Max', 'Std')
% set(1,'Color',[1 1 1]);
% 
% figure(2)
% plot((1:N)*opts.MaxIter, mean(maxScore), 'k-')
% title('Max path efficiency')
% xlabel('Iterations')
% ylabel('Path sensing efficiency')
% set(2,'Color',[1 1 1])
% 
% figure(3)
% plot((1:N)*opts.MaxIter, mean(distOfBest), 'k-')
% title('Dist of best leaf node')
% xlabel('Iterations')
% ylabel('Distance')
% set(3,'Color',[1 1 1])


% diary off;
% save 'testrun.mat';