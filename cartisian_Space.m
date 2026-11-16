%% set up
clear, clc, close all
% Define waypoints 
waypoints = [0.6  0.5  0    -0.8; 
            -0.2 -0.3 -0.25 -0.1; 
             0.6  0.3  1     0.65];
waypoints(:, 4) = waypoints(:, 1);
numWaypoints = size(waypoints, 2);

orientations = [0 0 0; pi/6 -pi/12 pi/8; pi/4 pi/12 -pi/8; pi/3 0 pi/4];
waypointTimes = [0 1 2 3]; 
waypointAccelTimes = [0.5 0.5 0.5];
ts = 0.01;
trajTimes = 0:ts:waypointTimes(end);
numSteps = numel(trajTimes);
trajType = 'quintic';
% 'quintic' or 'rottraj'
rentaiontrajectorytype = 'quintic'; 

% Boundary conditions for Cartesian motion
velocityBoundaryCondition = zeros(3, numWaypoints);
accelerationBoundaryCondition = zeros(3, numWaypoints);

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');
eeName = 'link7';
numJoints = numel(puma.homeConfiguration);
jointAnglesHome = [0 0 0 0 0 0]; 
for idx = 1:numJoints
    puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

% Define Inverse Kinematics solver
ik = inverseKinematics('RigidBodyTree', puma);

% Calculation Method: 'NumericalDifferentiation' or 'jacobian matrix'
calculatingmethod = 'NumericalDifferentiation';

% Thresholds & Damping parameters
angleThreshold = 0.000001;
velThreshold = 0.1;
lambda = 0.05;


plotMode = 1; 
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1), waypoints(2,1), waypoints(3,1), 'b.-');
end
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ro', 'LineWidth', 2);
%% Generate Cartesian Trajectory
[taskPos, taskVel, taskAcc, cmdOrient, ~, ~] = ...
    interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition, ...
    orientations, rentaiontrajectorytype, 1);

if plotMode == 1
    set(hTraj, 'xdata', taskPos(1,:), 'ydata', taskPos(2,:), 'zdata', taskPos(3,:));
    TrajPoints = plot3(waypoints(1,1), waypoints(2,1), waypoints(3,1), 'r.-');
end

%% Tracking 1
disp('Tracking Cartesian Trajectory with ikWeights = [0 0 0 1 1 1]...');
ikWeights_noOrient = [0 0 0 1 1 1];
[config_noOrient, ~] = followCartesianTrajectory(puma, eeName, ik, ikWeights_noOrient, ...
    jointAnglesHome, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, [], [], 0);
q_noOrient = config_noOrient';

% Compute actual task-space orientation from q using forward kinematics
[~, taskOrient_noOrient, ~, taskOrientVel_noOrient, ~, taskOrientAcc_noOrient] = ...
    computeTaskSpaceNumericalDifferentiation(puma, q_noOrient, trajTimes, eeName, angleThreshold);

eeVel_noOrient = [taskOrientVel_noOrient; taskVel];
eeAcc_noOrient = [taskOrientAcc_noOrient; taskAcc];

% Compute joint velocities, accelerations, and jerks based on calculatingmethod
if strcmpi(calculatingmethod, 'NumericalDifferentiation')
    jointVel_noOrient = [zeros(numJoints, 1), diff(q_noOrient, 1, 2) / ts];
    jointAcc_noOrient = [zeros(numJoints, 1), diff(jointVel_noOrient, 1, 2) / ts];
    jointJerks_noOrient = [zeros(numJoints, 1), diff(jointAcc_noOrient, 1, 2) / ts];
else
    [jointVel_noOrient, jointAcc_noOrient] = computeJointVelAccInverseJacobian(puma, ...
        q_noOrient, trajTimes, eeVel_noOrient, eeAcc_noOrient, eeName, lambda, angleThreshold, velThreshold);
    jointJerks_noOrient = [zeros(numJoints, 1), diff(jointAcc_noOrient, 1, 2) / ts];
    jointJerks_noOrient(abs(jointJerks_noOrient) < angleThreshold) = 0;
end

%% Tracking 2
disp('Tracking Cartesian Trajectory with ikWeights = [1 1 1 1 1 1]...');
ikWeights_orient = [1 1 1 1 1 1];
[config_orient, ~] = followCartesianTrajectory(puma, eeName, ik, ikWeights_orient, ...
    jointAnglesHome, taskPos, trajTimes, waypoints, [], angleThreshold, [], cmdOrient, 1);
q_orient = config_orient';

% Compute actual task-space orientation from q using forward kinematics
[~, taskOrient_orient, ~, taskOrientVel_orient, ~, taskOrientAcc_orient] = ...
    computeTaskSpaceNumericalDifferentiation(puma, q_orient, trajTimes, eeName, angleThreshold);

eeVel_orient = [taskOrientVel_orient; taskVel];
eeAcc_orient = [taskOrientAcc_orient; taskAcc];

% Compute joint velocities, accelerations, and jerks based on calculatingmethod
if strcmpi(calculatingmethod, 'NumericalDifferentiation')
    jointVel_orient = [zeros(numJoints, 1), diff(q_orient, 1, 2) / ts];
    jointAcc_orient = [zeros(numJoints, 1), diff(jointVel_orient, 1, 2) / ts];
    jointJerks_orient = [zeros(numJoints, 1), diff(jointAcc_orient, 1, 2) / ts];
else
    [jointVel_orient, jointAcc_orient] = computeJointVelAccInverseJacobian(puma, ...
        q_orient, trajTimes, eeVel_orient, eeAcc_orient, eeName, lambda, angleThreshold, velThreshold);
    jointJerks_orient = [zeros(numJoints, 1), diff(jointAcc_orient, 1, 2) / ts];
    jointJerks_orient(abs(jointJerks_orient) < angleThreshold) = 0;
end

%% Joint-Space Comparison Plots
labels = {'ikWeights = [0 0 0 1 1 1]', 'ikWeights = [1 1 1 1 1 1]'};
plotJointComparison(trajTimes, q_noOrient, q_orient, waypointTimes, labels);
plotJointSpaceTrajectories(trajTimes, {q_noOrient, q_orient}, ...
    {jointVel_noOrient, jointVel_orient}, ...
    {jointAcc_noOrient, jointAcc_orient}, ...
    [], waypointTimes, labels, {'b', 'r'}, ...
    {jointJerks_noOrient, jointJerks_orient});

%% Task-Space Trajectory Plots

eeVel_noOrient = [taskOrientVel_noOrient; taskVel];
eeAcc_noOrient = [taskOrientAcc_noOrient; taskAcc];
plots(trajTimes, taskPos, taskOrient_noOrient, eeVel_noOrient, eeAcc_noOrient, ...
    [], numJoints, ts, waypoints, 'ikWeights = [0 0 0 1 1 1]');

eeVel_orient = [taskOrientVel_orient; taskVel];
eeAcc_orient = [taskOrientAcc_orient; taskAcc];
plots(trajTimes, taskPos, taskOrient_orient, eeVel_orient, eeAcc_orient, ...
    [], numJoints, ts, waypoints, 'ikWeights = [1 1 1 1 1 1]');
