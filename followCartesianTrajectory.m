function [config_all, ikInitGuess] = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, tolerance, taskOrient, activateorentation)
% FOLLOWCARTESIANTRAJECTORY Solves inverse kinematics (IK) step-by-step along
% a Cartesian trajectory and animates the robot following the path.
% Supports both position-only and 6-DOF (position + orientation) Cartesian tracking.
%
% Syntax:
%   config_all = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints)
%   config_all = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints)
%   config_all = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, tolerance)
%   config_all = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, tolerance, taskOrient)
%   config_all = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, tolerance, taskOrient, activateorentation)
%   [config_all, ikInitGuess] = followCartesianTrajectory(...)
%
% Inputs:
%   puma               - RigidBodyTree robot model (e.g., puma560)
%   eeName             - Name of end-effector body (e.g., 'link7')
%   ik                 - inverseKinematics System object configured for the robot
%   ikWeights          - 1x6 vector of weights for [orientation, position]
%   ikInitGuess        - 1xN vector of initial joint angles configuration
%   taskPos            - 3xN matrix of Cartesian trajectory positions [x; y; z]
%   trajTimes          - 1xN vector of trajectory time stamps
%   waypoints          - 3xM matrix of Cartesian waypoints
%   TrajPoints         - (Optional) Line handle for traced path plot
%   angleThreshold     - (Optional) Absolute threshold below which joint angles are set to 0 (default: 1e-6)
%   tolerance          - (Optional) Distance tolerance to match waypoints (default: 1e-3)
%   taskOrient         - (Optional) 3xN matrix of Euler angles [roll; pitch; yaw] or Mx3 waypoint orientations
%   activateorentation - (Optional) Flag (1/0) to enable/disable orientation tracking (default: auto from taskOrient)
%
% Outputs:
%   config_all         - N x numJoints matrix of joint configurations over time
%   ikInitGuess        - Final joint configuration reached by the solver

    if nargin < 11 || isempty(tolerance)
        tolerance = 1e-3;
    end

    if nargin < 10 || isempty(angleThreshold)
        angleThreshold = 1e-6;
    end

    numSteps = numel(trajTimes);
    numJoints = numel(puma.homeConfiguration);

    % Handle orientation activation
    if nargin >= 13 && ~isempty(activateorentation)
        activateorentation = logical(activateorentation);
    else
        activateorentation = (nargin >= 12 && ~isempty(taskOrient));
    end

    if activateorentation && ~isempty(taskOrient)
        if size(taskOrient, 1) ~= 3 && size(taskOrient, 2) == 3
            taskOrient = taskOrient';
        end
        if size(taskOrient, 2) ~= numSteps
            % Waypoint orientations passed -> interpolate across trajTimes
            wpTimes = linspace(trajTimes(1), trajTimes(end), size(taskOrient, 2));
            taskOrient = quinticpolytraj(taskOrient, wpTimes, trajTimes);
        end
    end

    hasTrajPoints = (nargin >= 9 && ~isempty(TrajPoints) && isgraphics(TrajPoints));

    config_all = zeros(numSteps, numJoints);
    config = ikInitGuess;

    for idx = 1:numSteps
        % Solve IK with position and optional orientation
        if activateorentation && ~isempty(taskOrient)
            tgtPose = trvec2tform(taskPos(:, idx)') * eul2tform(taskOrient(:, idx)');
        else
            tgtPose = trvec2tform(taskPos(:, idx)');
        end
        [config, info] = ik(eeName, tgtPose, ikWeights, ikInitGuess);

        % Update initial guess for continuous tracking along trajectory
        ikInitGuess = config;
        config(abs(config) < angleThreshold) = 0;
        config_all(idx, :) = config;

        if ~strcmp(info.Status, 'success')
            warning('IK solver failed at time step %d', idx);
        end

        eePos = taskPos(:, idx)';
        if hasTrajPoints
            set(TrajPoints, 'xdata', [TrajPoints.XData, eePos(1)], ...
                            'ydata', [TrajPoints.YData, eePos(2)], ...
                            'zdata', [TrajPoints.ZData, eePos(3)]);
            set(TrajPoints, 'Color', 'r'); % Set trajectory trace color
        end

        % Check if eePos is close to any waypoint
        isMatch = any(all(abs(waypoints' - taskPos(:, idx)') < tolerance, 2));

        if isMatch
            % Find Cartesian points for visualization
            eeTform = getTransform(puma, config, eeName);
            plotTransforms(tform2trvec(eeTform), tform2quat(eeTform), 'FrameSize', 0.1);
        end

        % Show the robot
        show(puma, config_all(idx, :), 'Frames', 'off', 'PreservePlot', false);
        title(['Trajectory at t = ', num2str(trajTimes(idx))]);
        drawnow;
    end
end
