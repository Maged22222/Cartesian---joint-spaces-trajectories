%% Comparison: rottraj vs. Quintic Orientation Trajectories
% This script compares Cartesian orientation generation between:
%   1) 'quintic' - Independent quintic polynomial interpolation of Euler angles
%   2) 'rottraj' - SO(3) geodesic quaternion rotation interpolation
% Both trajectories use the exact same position waypoints and orientation targets.
% Tracking is performed using Inverse Kinematics, and joint velocities/accelerations
% are computed solely via numerical differentiation (NO Jacobian method used).

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
ikWeights = [1 1 1 1 1 1]; % Full 6-DOF tracking (Position + Orientation)
angleThreshold = 1e-6;

% Define Waypoints (Cartesian Positions)
waypoints = [0.6 0.5 0 -0.8; -0.2 -0.3 -0.25 -0.1; 0.6 0.3 1 0.65];
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

% Define Boundary Conditions for Cartesian Motion
velocityBoundaryCondition = zeros(3, numWaypoints);
accelerationBoundaryCondition = zeros(3, numWaypoints);

%% 2. Generate Trajectory 1: Quintic Euler Orientation
disp('Generating Quintic Euler Orientation Trajectory...');
[taskPos1, taskVel1, taskAcc1, cmdOrient_quintic, cmdOrientVel_quintic, cmdOrientAcc_quintic] = ...
    interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition, ...
    orientations, 'quintic');

%% 3. Follow Trajectory 1 (Quintic) with Robot
disp('Tracking Quintic Orientation trajectory with robot...');
[config_quintic, ~] = followCartesianTrajectory(puma, eeName, ik, ikWeights, ...
    jointAnglesHome, taskPos1, trajTimes, waypoints, [], angleThreshold, [], cmdOrient_quintic);

q_quintic = config_quintic';
jointVel_quintic = [zeros(numJoints, 1), diff(q_quintic, 1, 2) / ts];
jointAcc_quintic = [zeros(numJoints, 1), diff(jointVel_quintic, 1, 2) / ts];
jointJerk_quintic = [zeros(numJoints, 1), diff(jointAcc_quintic, 1, 2) / ts];

%% 4. Generate Trajectory 2: rottraj SO(3) Orientation
disp('Generating rottraj SO(3) Orientation Trajectory...');
[taskPos2, taskVel2, taskAcc2, cmdOrient_rottraj, cmdOrientVel_rottraj, cmdOrientAcc_rottraj] = ...
    interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition, ...
    orientations, 'rottraj');

%% 5. Follow Trajectory 2 (rottraj) with Robot
disp('Tracking rottraj Orientation trajectory with robot...');
[config_rottraj, ~] = followCartesianTrajectory(puma, eeName, ik, ikWeights, ...
    jointAnglesHome, taskPos2, trajTimes, waypoints, [], angleThreshold, [], cmdOrient_rottraj);

q_rottraj = config_rottraj';
jointVel_rottraj = [zeros(numJoints, 1), diff(q_rottraj, 1, 2) / ts];
jointAcc_rottraj = [zeros(numJoints, 1), diff(jointVel_rottraj, 1, 2) / ts];
jointJerk_rottraj = [zeros(numJoints, 1), diff(jointAcc_rottraj, 1, 2) / ts];

%% 6. Task-Space Orientation Comparison Plot (Direct Euler & Angular Velocity)
figure('Name', 'Task-Space Orientation Comparison: rottraj vs Quintic', 'NumberTitle', 'off');

subplot(2, 1, 1);
hold on;
plot(trajTimes, cmdOrient_quintic(1, :), 'b-', 'LineWidth', 1.5);
plot(trajTimes, cmdOrient_quintic(2, :), 'b--', 'LineWidth', 1.5);
plot(trajTimes, cmdOrient_quintic(3, :), 'b:', 'LineWidth', 1.5);
plot(trajTimes, cmdOrient_rottraj(1, :), 'r-', 'LineWidth', 1.5);
plot(trajTimes, cmdOrient_rottraj(2, :), 'r--', 'LineWidth', 1.5);
plot(trajTimes, cmdOrient_rottraj(3, :), 'r:', 'LineWidth', 1.5);
for wIdx = 1:numWaypoints
    xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
end
title('Task-Space Orientation (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Orientation (rad)');
legend('Roll (Quintic)', 'Pitch (Quintic)', 'Yaw (Quintic)', ...
       'Roll (rottraj)', 'Pitch (rottraj)', 'Yaw (rottraj)', 'Location', 'best');
grid on;

subplot(2, 1, 2);
hold on;
plot(trajTimes, cmdOrientVel_quintic(1, :), 'b-', 'LineWidth', 1.5);
plot(trajTimes, cmdOrientVel_quintic(2, :), 'b--', 'LineWidth', 1.5);
plot(trajTimes, cmdOrientVel_quintic(3, :), 'b:', 'LineWidth', 1.5);
plot(trajTimes, cmdOrientVel_rottraj(1, :), 'r-', 'LineWidth', 1.5);
plot(trajTimes, cmdOrientVel_rottraj(2, :), 'r--', 'LineWidth', 1.5);
plot(trajTimes, cmdOrientVel_rottraj(3, :), 'r:', 'LineWidth', 1.5);
for wIdx = 1:numWaypoints
    xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
end
title('Task-Space Angular Velocity (\omega_x, \omega_y, \omega_z)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('\omega_x (Quintic)', '\omega_y (Quintic)', '\omega_z (Quintic)', ...
       '\omega_x (rottraj)', '\omega_y (rottraj)', '\omega_z (rottraj)', 'Location', 'best');
grid on;

%% 7. Joint Position Comparison Over Time (Direct Comparison)
labels = {'Quintic Euler Orientation', 'rottraj SO(3) Orientation'};
plotJointComparison(trajTimes, q_quintic, q_rottraj, waypointTimes, labels);

%% 8. Joint-Space Profiles Comparison (Position, Velocity, Acceleration, Jerk)
plotJointSpaceTrajectories(trajTimes, {q_quintic, q_rottraj}, ...
    {jointVel_quintic, jointVel_rottraj}, ...
    {jointAcc_quintic, jointAcc_rottraj}, ...
    [], waypointTimes, labels, {'b', 'r'}, ...
    {jointJerk_quintic, jointJerk_rottraj});

disp('rottraj vs Quintic comparison completed successfully.');
