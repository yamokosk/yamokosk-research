function planner_demo( directory )
%% store current directory
currDir = pwd;

%% Go to problem directory
cd(directory)

%% Load target data
load traj.mat

%% Setup problem
def = readProblemDef( 'problem.def' );
x0 = def.ngen(targets(:,1));