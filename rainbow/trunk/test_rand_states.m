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
    
    plot_roboop(r1,xr(2:4,1),r2,xr(8:10,1)); drawnow;
    sceneSetVars({'q_src_2'; 'q_src_3'; 'q_src_5'; 'q_sen_2'; 'q_sen_3'; 'q_sen_5'}, [xr(2:4,1); xr(8:10,1)] );
    sceneRender();
    pause;
end