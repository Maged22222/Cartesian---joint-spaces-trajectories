function plotJointComparison(trajTimes, q1, q2, waypointTimes, labels, numJoints)
% PLOTJOINTCOMPARISON Plots comparative joint position trajectories for each joint.
%
% Syntax:
%   plotJointComparison(trajTimes, q1, q2, waypointTimes)
%   plotJointComparison(trajTimes, q1, q2, waypointTimes, labels)
%   plotJointComparison(trajTimes, q1, q2, waypointTimes, labels, numJoints)
%
% Inputs:
%   trajTimes     - 1 x N vector of trajectory timestamps
%   q1            - numJoints x N matrix of joint positions (case 1)
%   q2            - numJoints x N matrix of joint positions (case 2)
%   waypointTimes - 1 x M vector of waypoint arrival times
%   labels        - (Optional) 1 x 2 cell array of legend labels 
%                   (default: {'includeOrientation = false', 'includeOrientation = true'})
%   numJoints     - (Optional) Number of joints (default: size(q1, 1))

    if nargin < 5 || isempty(labels)
        labels = {'includeOrientation = false', 'includeOrientation = true'};
    end

    if nargin < 6 || isempty(numJoints)
        numJoints = size(q1, 1);
    end

    numWaypoints = numel(waypointTimes);

    for i = 1:numJoints
        figure;
        hold on;
        plot(trajTimes, q1(i, :), 'LineWidth', 1.5);
        plot(trajTimes, q2(i, :), 'LineWidth', 1.5);

        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
        end

        title(['Joint ', num2str(i), ' Position']);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        legend(labels{1}, labels{2});
        grid on;
    end
end
