%% Setup
clear, clc, close all
%trajType = input('Enter trajectory type (trap, cubic, quintic, bspline): ', 's');
% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 -0.3 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypoints(:,4)=waypoints(:,1);
orientations = [0 0 0; pi/6 -pi/12 pi/8; pi/4 pi/12 -pi/8; pi/3 0 pi/4];
waypointTimes = [0 1 2 3]; % Time at each waypoint
waypointAccelTimes = [0.5 0.5 0.5]; % Acceleration time for each segment
trajType = 'quintic';
% Define nonzero velocity boundary conditions for all joints
velocityBoundaryCondition = [0 0.5 0.5 0;  % Joint 1 velocities
                            0 1.0 1.0 0;  % Joint 2 velocities
                            0 1.5 1.5 0;  % Joint 3 velocities
                            0 0.2 0.2 0;  % Joint 4 velocities
                            0 0.1 0.1 0;  % Joint 5 velocities
                            0 0 0 0];     % Joint 6 velocities

% Define nonzero acceleration boundary conditions for all joints
accelerationBoundaryCondition = [0 0.2 0.2 0;  % Joint 1 accelerations
                                0 0.4 0.4 0;  % Joint 2 accelerations
                                0 0.6 0.6 0;  % Joint 3 accelerations
                                0 0.1 0.1 0;  % Joint 4 accelerations
                                0 0.05 0.05 0; % Joint 5 accelerations
                                0 0 0 0];     % Joint 6 accelerations

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Use a small sample time for this example
ts = 0.01;
trajTimes = 0:ts:waypointTimes(end);

% Define threshold for small angles
angleThreshold = 0.0001;


% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikInitGuess = jointAnglesHome;
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
plotMode = 1; 
show(puma,puma.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Solve IK for all waypoints and includeOrientation = false
includeOrientation = false; % Set this to use zero vs. nonzero orientations
ikWeights = [0 0 0 1 1 1];
numWaypoints = size(waypoints,2);
numJoints = numel(puma.homeConfiguration);

[jointWaypoints, ikInitGuess] = computeJointWaypoints(ik, eeName, waypoints, orientations, includeOrientation, ikWeights, ikInitGuess);
jointWaypoints_noOrient = jointWaypoints;

%% Generate trajectory on joint space


[q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes, angleThreshold);
%% Trajectory following loop
animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj);

%% Plot task-space trajectory for includeOrientation = false (using jacobian matrix)
% Compute task-space position, orientation, velocity, and acceleration using Jacobian
[eePos, eeOrient, eeVel, eeAcc] = computeTaskSpaceWithJacobian(puma, q, qd, qdd, eeName, angleThreshold);
% Plot position, orientation, velocity, acceleration, and jerks
plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints);

%% Plot task-space trajectory for includeOrientation = false (numerical differentiation)

% Compute task-space kinematics using numerical differentiation
[eePositions, eeOrientations, eeVelocities, eeAngularVelocities, ...
 eeAccelerations, eeAngularAccelerations, trajTimesVel, trajTimesAcc] = ...
 computeTaskSpaceNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold);

eeVel_num = [eeAngularVelocities; eeVelocities];
eeAcc_num = [eeAngularAccelerations; eeAccelerations];
plots(trajTimes, eePositions, eeOrientations, eeVel_num, eeAcc_num, qdd, numJoints, ts, waypoints, 'End-Effector Task-Space Trajectory (includeOrientation = false)');
%%  task space graphs preperation for includeOrientation = true
includeOrientation = true;
ikWeights = [1 1 1 1 1 1];
[jointWaypoints, ikInitGuess] = computeJointWaypoints(ik, eeName, waypoints, orientations, includeOrientation, ikWeights, ikInitGuess);
jointWaypoints_orient = jointWaypoints;
[qf, qfd, qfdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes, angleThreshold);                          
% Compute task-space kinematics using numerical differentiation (includeOrientation = true)
[eePositions2, eeOrientations2, eeVelocities2, eeAngularVelocities2, ...
 eeAccelerations2, eeAngularAccelerations2] = ...
 computeTaskSpaceNumericalDifferentiation(puma, qf, trajTimes, eeName, angleThreshold);
%% Plot task-space trajectory for includeOrientation = true (numerical differentiation)
eeVel_num2 = [eeAngularVelocities2; eeVelocities2];
eeAcc_num2 = [eeAngularAccelerations2; eeAccelerations2];
plots(trajTimes, eePositions2, eeOrientations2, eeVel_num2, eeAcc_num2, qfdd, numJoints, ts, waypoints, 'End-Effector Task-Space Trajectory (includeOrientation = true)');
%% results for joint space and task space
plotJointComparison(trajTimes, q, qf, waypointTimes);

%% Joint-Space Comparison: includeOrientation = false vs includeOrientation = true
labelsJoint = {'includeOrientation = false', 'includeOrientation = true'};
jointJerks_noOrient = [zeros(numJoints, 1), diff(qdd, 1, 2) / ts];
jointJerks_orient = [zeros(numJoints, 1), diff(qfdd, 1, 2) / ts];

plotJointSpaceTrajectories(trajTimes, {q, qf}, {qd, qfd}, {qdd, qfdd}, ...
    {jointWaypoints_noOrient, jointWaypoints_orient}, waypointTimes, ...
    labelsJoint, {'b', 'r'}, {jointJerks_noOrient, jointJerks_orient});