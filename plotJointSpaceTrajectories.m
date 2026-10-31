function plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor)
% PLOTJOINTSPACETRAJECTORIES Plots position, velocity, and acceleration for each joint.
%
% Syntax:
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor)
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

    for i = 1:numJoints
        figure;

        % Plot joint position
        subplot(3, 1, 1);
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
        subplot(3, 1, 2);
        hold on;
        plot(trajTimes, qd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--');
        end
        title(['Joint ', num2str(i), titleSuffix, ' Velocity']);
        xlabel('Time (s)');
        ylabel('Velocity (rad/s)');
        legend('Trajectory', 'Waypoints');
        grid on;

        % Plot joint acceleration
        subplot(3, 1, 3);
        hold on;
        plot(trajTimes, qdd(i, :), 'Color', lineColor, 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--');
        end
        title(['Joint ', num2str(i), titleSuffix, ' Acceleration']);
        xlabel('Time (s)');
        ylabel('Acceleration (rad/s^2)');
        legend('Trajectory', 'Waypoints');
        grid on;
    end
end
