%% Generate desired trajectory
freq = 15;
tf = 1/freq;
t = 0:tf:2;
vx = 0.75 * ones(size(t));
vy = zeros(size(t));%(0.075 * 2 * pi/0.75) * cos(2*pi*t/0.75);
x = vx.*t - 0.75;
y = zeros(size(t)); %0.075 * sin(2*pi*t/0.75);
[r1,r2] = planar_load_robots();

%% Set up figure
fig = figure(); hold on;
plot(x,y, 'o-');
set(gca, 'DataAspectRatio',   [1, 1, 1]);

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

%% Compute points
counter = 1;
visited = zeros(1,udata.traj.n);
while counter < 200
    [qr,rind] = planar_rand_state(Prob);
    
    if (~isempty(qr))
        %srccolor = [0, 0, 0] + ((rind-1)/udata.traj.n)*[0, 1, 0];
        %sencolor = [0, 0, 0] + ((rind-1)/udata.traj.n)*[1, 0, 0];
        %T_w_src = fkine_planar_pa10(qr(2:4),r1);
        %plot(T_w_src(1,4), T_w_src(2,4), '.', 'MarkerEdgeColor',srccolor,...
        %                        'MarkerFaceColor',srccolor);
        %plot(xr(6), xr(7), '.', 'MarkerEdgeColor',sencolor,...
        %                        'MarkerFaceColor',sencolor);
        %line([xr(2); xr(6)], [xr(3); xr(7)], 'Color', srccolor);
        plot_planar_pa10(qr(2:4),r1,fig);
        plot_planar_pa10(qr(8:10),r2,fig);
        drawnow;
        visited(rind) = visited(rind) + 1;
        counter = counter + 1;
    end
end

figure();
bar(visited);