%% Setup problem
clear;

%% Generate desired trajectory
freq = 15;
dt = 1/freq;
t = 0:dt:6;
vx = 0.25 * ones(size(t));
vy = zeros(size(t));%(0.075 * 2 * pi/0.75) * cos(2*pi*t/0.75);
x = vx.*t - 0.75;
y = zeros(size(t)); %0.075 * sin(2*pi*t/0.75);
[r1,r2] = planar_load_robots(fullfile(cd, 'roboop', 'pa10_planar.conf'));

%% Set up figure
fig = figure(); hold on;
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
% counter = 1;
% x0 = []; smallest_rind = inf;
% while counter < 200
%     [xr,rind] = planar_rand_state(Prob);
%     
%     if (~isempty(xr))
%         if (rind < smallest_rind)
%             smallest_rind = rind;
%             x0 = xr;
%         end
%         counter = counter + 1;
%     end
% end
x0 = [0.6000,    0.6000; ...
   -0.2902,   -0.2902; ...
   -0.9122,   -0.9122; ...
    1.1529,    1.1529; ...
    0.3956,    0.3274; ...
   -0.5264,   -0.4485; ...
    0.6172,    0.5202; ...
    0.3920,    1.3074; ...
    0.8849,   -0.8849; ...
   -1.3263,   -0.4720; ...
   -0.3396,    0.0919; ...
    0.4477,   -0.3595; ...
   -0.7035,   -0.2012];
   
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