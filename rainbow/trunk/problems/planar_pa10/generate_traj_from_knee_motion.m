function generate_traj_from_knee_motion()

%% Load data knee motion data (in milimeters)
data = load('kneemotion.txt');

Ts_original = 0.0056;    % Sample time of original data in seconds
Ts_original = 10*Ts_original;

x = data(47:139,2);
y = data(47:139,1);
z = zeros(size(x));

%% Recenter data and convert to meters
x = (x - mean(x)) * 0.001;
y = (y - mean(y)) * 0.001;
vx = [0; diff(x)]/Ts_original;
vy = [0; diff(y)]/Ts_original;

%% Down sample towards 25 Hz
fs_original = 1/Ts_original;
fs_desired = 1;
r = round(fs_original/fs_desired);
fs_actual = fs_original/r;

x_dec = decimate(x,r);
y_dec = decimate(y,r);
vx_dec = decimate(vx,r);
vy_dec = decimate(vy,r);


%% Create a time vector
nFrames = length(x_dec);
Ts_actual = 1/fs_actual;
t = (0:nFrames-1)*Ts_actual;

%% Create desired view vectors which are normal to velocity
u = zeros(2,nFrames);
for n = 1:nFrames
    ud = [-vy_dec(n); vx_dec(n)];
    ud_hat = ud / norm(ud);
    u(:,n) = ud_hat;
end

%% Plot to check this is what we want
% figure(1)
% plot(x_dec,y_dec,'o-');
% hold on;
% for n=1:nFrames
%     line([x_dec(n); x_dec(n)+u(1,n)], [y_dec(n); y_dec(n)+u(2,n)]);
% end
% axis([-1, 1, -1.5, 1.5]);

%% Save data to traj.mat
targets = [x_dec'; y_dec'; vx_dec'; vy_dec'; u; t];
save 'traj.mat' targets