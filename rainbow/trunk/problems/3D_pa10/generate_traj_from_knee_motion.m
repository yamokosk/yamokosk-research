function [t,pp_target] = generate_traj_from_knee_motion()

%% Load data knee motion data (in milimeters)
data = load('kneemotion.txt');

Ts_original = 0.0056;    % Sample time of original data in seconds

x = data(:,1) * 0.001;  % convert to meters
x = x - mean(x);
y = data(:,2) * 0.001;  % convert to meters
z = data(:,3) * 0.001;  % convert to meters

%% Create a time vector
nFrames = length(x);
t = (0:nFrames-1)*Ts_original;

%% Spline fit the data to generate smooth velocities
pp = interp1(t',[x, y, z], 'spline', 'pp');

% Break out coefficients of the fit to create the derivative.
[breaks,coefs,l,k,d] = unmkpp(pp);
dpp = mkpp(breaks,repmat(k-1:-1:1,d*l,1).*coefs(:,1:k-1),d);
 
pos = ppval(pp,t');
dpos = ppval(dpp,t')';

ori = [0, 1, 0] * pi/2;
ori = repmat(ori, nFrames, 1);
dori = zeros(nFrames,3);

target = [pos, ori, dpos, dori];
pp_target = interp1(t', target, 'spline', 'pp');