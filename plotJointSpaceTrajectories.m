function plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor, qddd)
% PLOTJOINTSPACETRAJECTORIES Plots position, velocity, and acceleration for each joint.
% Optionally plots jerk if qddd is provided.
%
% Syntax:
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor, qddd)
%
% Inputs:
%   trajTimes      - 1 x N vector of trajectory timestamps
%   q              - numJoints x N matrix of joint positions
%   qd             - numJoints x N matrix of joint velocities
%   qdd            - numJoints x N matrix of joint accelerations
%   jointWaypoints - (Optional) numJoints x numWaypoints matrix of waypoint joint configurations
%   waypointTimes  - 1 x numWaypoints vector of waypoint arrival times
%   titleSuffix    - (Optional) String suffix for titles (e.g., 'includeOrientation = false')
%   lineColor      - (Optional) Line color specification (default: 'b')
%   qddd           - (Optional) numJoints x N (or N x numJoints) matrix of joint jerks

    if nargin < 7 || isempty(titleSuffix)
        titleSuffix = '';
    else
        titleSuffix = ['  ', titleSuffix];
    end

    if nargin < 8 || isempty(lineColor)
        lineColor = 'b';
    end

    numJoints = size(q, 1);
    numWaypoints = numel(waypointTimes);

    hasJerk = (nargin >= 9 && ~isempty(qddd));
    if hasJerk && size(qddd, 1) ~= numJoints && size(qddd, 2) == numJoints
        qddd = qddd';
    end

    numSubplots = 3;
    if hasJerk
        numSubplots = 4;
    end

    for i = 1:numJoints
        figure;

        % Plot joint position
        subplot(numSubplots, 1, 1);
        hold on;
        plot(trajTimes, q(i, :), 'Color', lineColor, 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--');
        end
        if nargin >= 5 && ~isempty(jointWaypoints)
            plot(waypointTimes, jointWaypoints(i, :), 'ro', 'MarkerSize', 2);
            legend('Trajectory', 'Waypoints');
        else
            legend('Trajectory');
        end
        title(['Joint ', num2str(i), titleSuffix, ' Position']);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        grid on;

        % Plot joint velocity
        subplot(numSubplots, 1, 2);
        hold on;
        plot(trajTimes, qd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--');
        end
        title(['Joint ', num2str(i), titleSuffix, ' Velocity']);
        xlabel('Time (s)');
        ylabel('Velocity (rad/s)');
        legend('Trajectory');
        grid on;

        % Plot joint acceleration
        subplot(numSubplots, 1, 3);
        hold on;
        plot(trajTimes, qdd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--');
        end
        title(['Joint ', num2str(i), titleSuffix, ' Acceleration']);
        xlabel('Time (s)');
        ylabel('Acceleration (rad/s^2)');
        legend('Trajectory');
        grid on;

        % Plot joint jerk (if provided)
        if hasJerk
            subplot(numSubplots, 1, 4);
            hold on;
            plot(trajTimes, qddd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Jerk']);
            xlabel('Time (s)');
            ylabel('Jerk (rad/s^3)');
            legend('Trajectory');
            grid on;
        end
    end
end
