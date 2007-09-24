function [pp_gait, conrad_time] = loadGaitData()

% Get Conrad's data
deg2rad = pi/180; %unitsratio('rad','deg');
%m2in = unitsratio('in','m');

pelvis_trans = load('gaitdata\PelvisTrans.txt')*1000;
pelvis_trans(:,3) = pelvis_trans(:,3) + .069;
pelvis_trans(:,2) = pelvis_trans(:,2) + .2;
%$pelvis_trans = [pelvis_trans(:,2), pelvis_trans(:,1), pelvis_trans(:,3)];
pelvis_rot = loadOSMIFile('gaitdata\PelvisRot.txt') * deg2rad;

rhip = loadOSMIFile('gaitdata\RHipAngle.txt') * deg2rad;
rknee = loadOSMIFile('gaitdata\RKneeAngle.txt') * deg2rad;
rankle = loadOSMIFile('gaitdata\RAnkleAngle.txt') * deg2rad;
rshoulder = loadOSMIFile('gaitdata\RShoulderAngle.txt') * deg2rad;
relbow = loadOSMIFile('gaitdata\RElbowAngle.txt') * deg2rad;

lhip = loadOSMIFile('gaitdata\LHipAngle.txt') * deg2rad;
lknee = loadOSMIFile('gaitdata\LKneeAngle.txt') * deg2rad;
lankle = loadOSMIFile('gaitdata\LAnkleAngle.txt') * deg2rad;
lshoulder = loadOSMIFile('gaitdata\LShoulderAngle.txt') * deg2rad;
lelbow = loadOSMIFile('gaitdata\LElbowAngle.txt') * deg2rad;

% Assuming that Conrad's data was taken at 60Hz
frames_per_second = 60;
conrad_num_frames = size(lhip,1);
conrad_time = linspace(0,conrad_num_frames / 60, conrad_num_frames);

% Need to interpolate to make them all match up
gait_data = [pelvis_trans, pelvis_rot, lhip, lknee, lankle, lshoulder, lelbow, rhip, rknee, rankle, rshoulder, relbow];
pp_gait = interp1(conrad_time, gait_data, 'spline', 'pp');

%clear frames_per_second conrad_num_frames pelvis_trans pelvis_rot lhip lknee lankle lshoulder lelbow rhip rknee rankle rshoulder relbow