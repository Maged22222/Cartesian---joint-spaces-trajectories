%% Trajectory Comparison: Joint Space vs. Cartesian Space
% This script compares trajectory generation, 3D robot animation, task-space
% kinematics, and joint-space dynamics for both Joint-Space and Cartesian-Space trajectories.
clear, clc, close all

%% 1. General Setup & Robot Parameters
% Load PUMA 560 robot model
puma = loadrobot('puma560', 'DataFormat', 'row');
eeName = 'link7';
numJoints = numel(puma.homeConfiguration);
jointAnglesHome = [0 0 0 0 0 0];

% Update joint limits to [-pi, pi] for all joints
for idx = 1:numJoints
    puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

% Define Inverse Kinematics solver
ik = inverseKinematics('RigidBodyTree', puma);
ikWeights = [1 1 1 1 1 1];
ikWeightsCart = [0 0 0 1 1 1]; % Position-only for Cartesian tracking
angleThreshold = 1e-4;

% Define Waypoints (Cartesian Positions)
waypoints = [0.6 0.5 0 -0.8; -0.2 0 -0.25 -0.1; 0.6 0.3 1 0.65];
waypoints(:, 4) = waypoints(:, 1); % Closed-loop path
numWaypoints = size(waypoints, 2);

% Define Waypoint Orientations (Roll, Pitch, Yaw)
orientations = [0 0 0; pi/6 -pi/12 pi/8; pi/4 pi/12 -pi/8; pi/3 0 pi/4];

% Define Time Parameters
waypointTimes = [0 1 2 3];
waypointAccelTimes = [0.5 0.5 0.5];
ts = 0.01;
trajTimes = 0:ts:waypointTimes(end);
numSteps = numel(trajTimes);
trajType = 'quintic';

% Define Boundary Conditions for Joint Space
jointVelBC = [0 0.5 0.5 0;
              0 1.0 1.0 0;
              0 1.5 1.5 0;
              0 0.2 0.2 0;
              0 0.1 0.1 0;
              0 0   0   0];

jointAccBC = [0 0.2  0.2  0;
              0 0.4  0.4  0;
              0 0.6  0.6  0;
              0 0.1  0.1  0;
              0 0.05 0.05 0;
              0 0    0    0];

% Define Boundary Conditions for Cartesian Space
cartVelBC = zeros(3, numWaypoints);
cartAccBC = zeros(3, numWaypoints);

taskOrientWaypoints = zeros(3, numSteps);
taskOrientVel = zeros(3, numSteps);
taskOrientAcc = zeros(3, numSteps);

%% 2. Generate Joint-Space Trajectory
disp('Computing Joint-Space Trajectory...');
includeOrientation = false;
[jointWaypoints, ikInitGuess] = computeJointWaypoints(ik, eeName, waypoints, orientations, ...
    includeOrientation, ikWeights, jointAnglesHome);

[q_joint, qd_joint, qdd_joint] = interpolateJointTrajectory(trajType, jointWaypoints, ...
    waypointTimes, trajTimes, waypointAccelTimes, angleThreshold);

% Compute Task-Space kinematics for Joint-Space Trajectory using Jacobian
[eePos_joint, eeOrient_joint, eeVel_joint, eeAcc_joint] = ...
    computeTaskSpaceWithJacobian(puma, q_joint, qd_joint, qdd_joint, eeName, angleThreshold);

% Compute Task-Space kinematics for Joint-Space Trajectory using Numerical Differentiation
[eePos_joint_num, eeOrient_joint_num, eeVel_joint_num, eeAngularVel_joint_num, ...
 eeAcc_joint_num, eeAngularAcc_joint_num] = ...
    computeTaskSpaceNumericalDifferentiation(puma, q_joint, trajTimes, eeName, angleThreshold);

%% 3. Generate Cartesian-Space Trajectory
disp('Computing Cartesian-Space Trajectory...');
[taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, ...
    waypointTimes, trajTimes, waypointAccelTimes, cartVelBC, cartAccBC);

%% 4. Animation 1: Joint-Space Trajectory Following
disp('Animating Joint-Space Trajectory...');
figure('Name', 'Joint-Space Trajectory Animation', 'NumberTitle', 'off');
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2]);
hold on;
hTraj_joint = plot3(waypoints(1,1), waypoints(2,1), waypoints(3,1), 'b.-');
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ro', 'LineWidth', 2);
title('Joint-Space Trajectory Animation');
animateRobotTrajectory(puma, q_joint, trajTimes, waypoints, eeName, hTraj_joint);

%% 5. Animation 2: Cartesian-Space Trajectory Following
disp('Animating Cartesian-Space Trajectory...');
figure('Name', 'Cartesian-Space Trajectory Animation', 'NumberTitle', 'off');
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2]);
hold on;
plot3(taskPos(1,:), taskPos(2,:), taskPos(3,:), 'b--');
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ro', 'LineWidth', 2);
TrajPoints_cart = plot3(waypoints(1,1), waypoints(2,1), waypoints(3,1), 'r.-');
title('Cartesian-Space Trajectory Animation');
[config_cart, ~] = followCartesianTrajectory(puma, eeName, ik, ikWeightsCart, ...
    jointAnglesHome, taskPos, trajTimes, waypoints, TrajPoints_cart, angleThreshold);

q_cart = config_cart';

%% 6. Direct 3D Path Comparison (Joint Space vs Cartesian Space)
figure('Name', '3D Trajectory Comparison', 'NumberTitle', 'off');
hold on;
plot3(eePos_joint(1, :), eePos_joint(2, :), eePos_joint(3, :), 'b-', 'LineWidth', 2);
plot3(taskPos(1, :), taskPos(2, :), taskPos(3, :), 'r--', 'LineWidth', 2);
plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
title('3D End-Effector Trajectory Comparison');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
legend('Joint-Space Path', 'Cartesian-Space Path', 'Waypoints');
view(3);

%% 7. Joint Position Comparison (Joint Space vs Cartesian Space)
plotJointComparison(trajTimes, q_joint, q_cart, waypointTimes, ...
    {'Joint-Space Trajectory', 'Cartesian-Space Trajectory'});

%% 8. Joint-Space Profiles for Joint-Space Trajectory
jointJerks_joint = [zeros(numJoints, 1), diff(qdd_joint, 1, 2) / ts];
plotJointSpaceTrajectories(trajTimes, q_joint, qd_joint, qdd_joint, jointWaypoints, ...
    waypointTimes, 'Joint-Space Trajectory', 'b', jointJerks_joint);

%% 9. Joint-Space Profiles for Cartesian-Space Trajectory (Numerical Diff vs Inverse Jacobian)
% Numerical Differentiation
jointVel_cart_num = [zeros(numJoints, 1), diff(q_cart, 1, 2) / ts];
jointAcc_cart_num = [zeros(numJoints, 1), diff(jointVel_cart_num, 1, 2) / ts];
jointJerk_cart_num = [zeros(numJoints, 1), diff(jointAcc_cart_num, 1, 2) / ts];

% Inverse Jacobian
eeVel_cart = [taskOrientVel; taskVel];
eeAcc_cart = [taskOrientAcc; taskAcc];
[qd_cart_jac, qdd_cart_jac] = computeJointVelAccInverseJacobian(puma, q_cart, trajTimes, eeVel_cart, eeAcc_cart, eeName);
jointJerk_cart_jac = [zeros(numJoints, 1), diff(qdd_cart_jac, 1, 2) / ts];

% Comparative Joint Plots for Cartesian Space
labelsCart = {'Numerical Differentiation', 'Inverse Jacobian'};
plotJointSpaceTrajectories(trajTimes, q_cart, ...
    {jointVel_cart_num, qd_cart_jac}, ...
    {jointAcc_cart_num, qdd_cart_jac}, ...
    [], waypointTimes, labelsCart, {'b', 'r'}, ...
    {jointJerk_cart_num, jointJerk_cart_jac});

%% 10. Task-Space Plots (Joint-Space Trajectory)
% Jacobian Task-Space
plots(trajTimes, eePos_joint, eeOrient_joint, eeVel_joint, eeAcc_joint, qdd_joint, ...
    numJoints, ts, waypoints, 'Joint-Space Task Trajectory (Jacobian)');

% Numerical Differentiation Task-Space
eeVel_joint_num_all = [eeAngularVel_joint_num; eeVel_joint_num];
eeAcc_joint_num_all = [eeAngularAcc_joint_num; eeAcc_joint_num];
plots(trajTimes, eePos_joint_num, eeOrient_joint_num, eeVel_joint_num_all, eeAcc_joint_num_all, ...
    qdd_joint, numJoints, ts, waypoints, 'Joint-Space Task Trajectory (Numerical Diff)');

%% 11. Task-Space Plots (Cartesian-Space Trajectory)
plots(trajTimes, taskPos, taskOrientWaypoints, eeVel_cart, eeAcc_cart, qdd_cart_jac, ...
    numJoints, ts, waypoints, 'Cartesian-Space Task Trajectory');

disp('Trajectory Comparison script execution completed successfully.');
