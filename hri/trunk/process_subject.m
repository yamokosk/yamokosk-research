function subject_data = process_subject(id)

%BW_in_lb = input('Enter subjects body weight [lb]: ');
%BW_in_N = BW_in_lb * 0.45359237 * 9.8;

%[filename, pathname, filterindex] = uigetfile( ...
%    {  '*.forces','Force files (*.forces)'; ...
%       '*.*',  'All Files (*.*)'}, ...
%       'Select non-robot trials', ...
%       'MultiSelect', 'on');

% Load t-pose to get subject's body weight
tPoseFileName = ls([id '\t_pose\*.forces']);
if ( size(tPoseFileName, 1) ~= 1 )
	error('No t-pose data found.');
end
[tPoseData, sampleRate] = load_forces_file( fullfile(id,'t_pose',tPoseFileName) );
tPoseData_bar = [];
if ( size(tPoseData,1) == 1 )
    tPoseData_bar = tPoseData(1,1:3,1);
else
    tPoseData_bar = mean(tPoseData(:,1:3,1));
end
body_weight = sqrt(tPoseData_bar(1,1)^2 + tPoseData_bar(1,2)^2 + tPoseData_bar(1,3)^2);

% Load and process non-robot (control) data
controlTrialFiles = ls([id '\no_robot\*.forces']);
fprintf(1, 'Reading control trials: ');
controlTrialAnalysis = analyzeForceData(body_weight, fullfile(id,'no_robot'), controlTrialFiles);

% Load and process robot (treatment) data
robotTrialFiles = ls([id '\with_robot\*.forces']);
fprintf(1, 'Reading robot trials: ');
robotTrialAnalysis = analyzeForceData(body_weight, fullfile(id,'with_robot'), robotTrialFiles);

% % Update Excel spreadsheet
% sheets = {'ML','AP','Vertical'};
% column = input('Enter column letter to store results: ','s');
% if ( isempty(column) )
%     column = 'B';
% end
% 
% for n = 1:length(sheets)
%     M = {id; ...
%          controlTrialAnalysis(1,n); ...
%          controlTrialAnalysis(2,n); ...
%          controlTrialAnalysis(3,n); ...
%          controlTrialAnalysis(4,n); ...
%          controlTrialAnalysis(5,n); ...
%          robotTrialAnalysis(1,n); ...
%          robotTrialAnalysis(2,n); ...
%          robotTrialAnalysis(3,n); ...
%          robotTrialAnalysis(4,n); ...
%          robotTrialAnalysis(5,n)};
%     xlswrite('Data analysis.xls', M, sheets{n}, [column '1']);
% end
subject_data(:,:,1) = controlTrialAnalysis;
subject_data(:,:,2) = robotTrialAnalysis;

function analysis = analyzeForceData(body_weight, rootdir, filenames)

numFiles = size(filenames, 1);
if ( numFiles ~= 5 )
    error('I was expecting 5 trials.');
end

analysis = zeros(numFiles, 3);
level = 0.05;
for n = 1:numFiles
    [trialData, sampleRate] = load_forces_file( fullfile(rootdir, filenames(n,:)) );
    normTrialData = trialData(:,1:3,1)/body_weight;
    [croppedTrialData, level, N] = extract_stance_phase(normTrialData, level);
    [path,name] = fileparts(filenames(n,:));
    fprintf(1, '%s (%d) ', name, N);
    for m = 1:3
        [Pxx,freq] = compute_signal_psd(croppedTrialData(:,m),sampleRate);
        analysis(n,m) = compute_signal_bandwidth(freq, Pxx);
    end
end
fprintf(1, '\n');

% 
% numFiles = 
% [junk, numNoRobotTrials] = size(filename);
% NoRobotTrialData = zeros(numNoRobotTrials,3);
% for n = 1:numNoRobotTrials
%     [raw_data,sampleRate] = load_forces_file(filename{n});
%     normalized = raw_data(:,1:3,1)/BW_in_N;
%     stance_phase = extract_stance_phase(normalized);
%     for m = 1:3
%         [Pxx,freq] = compute_signal_psd(stance_phase(:,m),sampleRate);
%         NoRobotTrialData(n,m) = compute_signal_bandwidth(freq, Pxx);
%         %NoRobotTrialData(n,m) = compute_signal_bandwidth(stance_phase(:,m),sampleRate);
%     end
% end
% 
% [filename, pathname, filterindex] = uigetfile( ...
%     {  '*.forces','Force files (*.forces)'; ...
%        '*.*',  'All Files (*.*)'}, ...
%        'Select robot trials', ...
%        'MultiSelect', 'on');
% [junk, numRobotTrials] = size(filename);
% RobotTrialData = zeros(numRobotTrials,3);
% for n = 1:numNoRobotTrials
%     [raw_data,sampleRate] = load_forces_file(filename{n});
%     normalized = raw_data(:,1:3,1)/BW_in_N;
%     stance_phase = extract_stance_phase(normalized);
%     for m = 1:3
%         [Pxx,freq] = compute_signal_psd(stance_phase(:,m),sampleRate);
%         RobotTrialData(n,m) = compute_signal_bandwidth(freq, Pxx);
%     end
% end
% 
% NoRobotTrialData
% 
% xforce_no_robot = [mean(NoRobotTrialData(:,1)), std(NoRobotTrialData(:,1))]
% yforce_no_robot = [mean(NoRobotTrialData(:,2)), std(NoRobotTrialData(:,2))]
% zforce_no_robot = [mean(NoRobotTrialData(:,3)), std(NoRobotTrialData(:,3))]
% 
% RobotTrialData
% 
% xforce_robot = [mean(RobotTrialData(:,1)), std(RobotTrialData(:,1))]
% yforce_robot = [mean(RobotTrialData(:,2)), std(RobotTrialData(:,2))]
% zforce_robot = [mean(RobotTrialData(:,3)), std(RobotTrialData(:,3))]
