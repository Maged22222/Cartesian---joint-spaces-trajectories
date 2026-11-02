%% Setup
clear, clc, close all

% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 -0.3 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypoints(:,4)=waypoints(:,1);
waypointTimes = [0 1 2 3]; % Time at each waypoint
waypointAccelTimes = [0.5 0.5 0.5]; % Acceleration time for each segment
numWaypoints = size(waypoints,2);
% Use a small sample time for this example
ts = 0.01;
trajTimes = 0:ts:waypointTimes(end);
trajType = 'quintic';

% Define velocity boundary conditions for Cartesian motion (X, Y, Z for each waypoint)
velocityBoundaryCondition = [0 0 0 0;   % Vx at waypoints
                             0 0 0 0;   % Vy at waypoints
                             0 0 0 0];  % Vz at waypoints

% Define acceleration boundary conditions for Cartesian motion (X, Y, Z for each waypoint)
accelerationBoundaryCondition = [0 0 0 0;   % Ax at waypoints
                                 0 0 0 0;   % Ay at waypoints
                                 0 0 0 0];  % Az at waypoints
taskOrientWaypoints =zeros(3,numel(trajTimes));
taskOrientVel = zeros(3, numel(trajTimes)); % Angular velocity for all time steps
taskOrientAcc = zeros(3, numel(trajTimes)); % Angular acceleration for all time

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Define threshold for small angles
angleThreshold = 0.000001;
% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikWeights = [0 0 0 1 1 1];
ikInitGuess =jointAnglesHome;
numJoints = numel(puma.homeConfiguration);

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Changing Joint Limits
for idx = 1:numJoints
 
  puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

%% Generate trajectory
% Cartesian Motion only
[taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition);

% Show the full trajectory with the rigid body tree
if plotMode == 1
    set(hTraj,'xdata',taskPos(1,:),'ydata',taskPos(2,:),'zdata',taskPos(3,:));
    TrajPoints = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'r.-');
elseif plotMode == 2
    plotTransforms(taskPos',repmat([1 0 0 0],[size(taskPos,2) 1]),'FrameSize',0.05);
end

%% Trajectory following loop
if ~exist('TrajPoints', 'var')
    TrajPoints = [];
end
[config_all, ikInitGuess] = followCartesianTrajectory(puma, eeName, ik, ikWeights, ...
    ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold);




numSteps = numel(trajTimes);
q = config_all';    % Joint positions
numJoints = size(q, 1);
qd = zeros(numJoints, numSteps);   % Joint velocities
qdd = zeros(numJoints, numSteps);  % Joint accelerations

% Compute joint velocities, accelerations, and jerks (numerical differentiation)
jointVelocities = [zeros(numJoints, 1), diff(q, 1, 2) / ts];
jointAccelerations = [zeros(numJoints, 1), diff(jointVelocities, 1, 2) / ts];
jointJerks = [zeros(numJoints, 1), diff(jointAccelerations, 1, 2) / ts];

%% Compute Joint Velocities and Accelerations Using Inverse Jacobian and check singularity
eeVel = [taskOrientVel; taskVel];
eeAcc = [taskOrientAcc; taskAcc];
[qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName);

% Compute joint jerks from Inverse Jacobian accelerations
jointJerksJac = [zeros(numJoints, 1), diff(qdd, 1, 2) / ts];

%% Joint-Space Comparison Plots (Numerical Differentiation vs Inverse Jacobian)
labels = {'Numerical Differentiation', 'Inverse Jacobian'};
plotJointSpaceTrajectories(trajTimes, q, ...
    {jointVelocities, qd}, ...
    {jointAccelerations, qdd}, ...
    [], waypointTimes, labels, {'b', 'r'}, ...
    {jointJerks, jointJerksJac});


%% Plot Task-Space Trajectory (Position, Orientation, Velocity, Acceleration, Jerk, 3D Path)
plots(trajTimes, taskPos, taskOrientWaypoints, eeVel, eeAcc, qdd, numJoints, ts, waypoints, 'Cartesian Task-Space Trajectory');