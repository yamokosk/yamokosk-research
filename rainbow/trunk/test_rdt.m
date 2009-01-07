%% Setup problem
clear;

%% Generate target trajectory
freq = 5;
dt = 1/freq;
t = 0:dt:6; numTargets = length(t);
vx = 0.25 * ones(size(t));
vy = (0.075 * 2 * pi/0.75) * cos(2*pi*t/0.75);

% Target positions
x = vx.*t - 0.75;
y = 0.075 * sin(2*pi*t/0.75);

% Desired vantage points for targets
ux = zeros(1,numTargets);
uy = ones(1,numTargets);

targets = [x; y; ux; uy; t];

%% Define problem
Prob = plannerAssign( fullfile(pwd,'problems','two_dof_system') );
Prob.x0 = Prob.func_handles.ngen(targets(:,1), Prob.userdata);

%% Solve
Prob = plannerSolve(Prob);

%% Set up figure
fig = plot_roboop(r1);
plot_roboop(r2); hold on;
plot(x,y, 'o-');
set(gca, 'DataAspectRatio',   [1, 1, 1]);
p = [-.1, -.2]
r=0.05
plot(r*cos(t)+p(1),r*sin(t)+p(2),'g-')
p = [.1,.3]
r = 0.075
plot(r*cos(t)+p(1),r*sin(t)+p(2),'g-')

%% Store in userdata section
udata.traj.n = length(x);
udata.traj.t = t;
udata.traj.x = x;
udata.traj.y = y;
udata.traj.vx = vx;
udata.traj.vy = vy;
udata.r1 = r1;
udata.r2 = r2;
Prob.userdata = udata;

%% Generate a bunch of points to find the left most starting point
counter = 1;
x0 = []; smallest_rind = inf;
while counter < 200
    [xr,rind] = planar_rand_state_obs(Prob);
    
    if (~isempty(xr))
        if (rind < smallest_rind)
            smallest_rind = rind;
            x0 = xr;
        end
        counter = counter + 1;
    end
end
   
%% Call rdtAssign
x_lb = [t(1); r1.qmin'; -r1.qpmax'; r2.qmin'; -r2.qpmax'];
x_ub = [t(end); r1.qmax'; r1.qpmax'; r2.qmax'; r2.qpmax'];
u_lb = -1*[r1.umax'; r2.umax'];
u_ub = [r1.umax'; r2.umax'];
Prob = rdtAssign(x0, [], x_lb, x_ub, u_lb, u_ub, 100, ...
                 'OBSFREE', [], udata, ...
                 @planar_robots_ode, ...
                 @planar_rand_state_obs, ...
                 @planar_state_eval, ...
                 @planar_node_select, ...
                 @planar_local_planner_obs, ...
                 @rdtOutput);

%% Call RDT
tic;
%for n = 1:10000
    Prob = rdtSolve(Prob);
%end
Prob.runtime = toc;

%% Save results
str = datestr(now,30);
dirname = [Prob.name '_' datestr(now, 30)];
currdir = cd;
status = mkdir(dirname);

if (status)
    cd(dirname);
    save('data', 'Prob');
    for n = 1:1
        hgsave(n,['Fig' num2str(n)]);
    end
    cd(currdir);
else
    error('Could not save results of simulation!');    
end