function plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor, qddd)
% PLOTJOINTSPACETRAJECTORIES Plots position, velocity, and acceleration for each joint.
% Optionally plots jerk if qddd is provided.
% Supports both single trajectory plotting and comparative overlay plotting.
%
% Syntax (Single Trajectory):
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes)
%   plotJointSpaceTrajectories(trajTimes, q, qd, qdd, jointWaypoints, waypointTimes, titleSuffix, lineColor, qddd)
%
% Syntax (Comparative Mode - e.g. Numerical Diff vs Inverse Jacobian):
%   plotJointSpaceTrajectories(trajTimes, q, {qd1, qd2}, {qdd1, qdd2}, [], waypointTimes, labels, {'b', 'r'}, {qddd1, qddd2})
%
% Inputs:
%   trajTimes      - 1 x N vector of trajectory timestamps
%   q              - numJoints x N matrix (or {q1, q2} cell array) of joint positions
%   qd             - numJoints x N matrix (or {qd1, qd2} cell array) of joint velocities
%   qdd            - numJoints x N matrix (or {qdd1, qdd2} cell array) of joint accelerations
%   jointWaypoints - (Optional) numJoints x numWaypoints matrix of waypoint joint configurations
%   waypointTimes  - 1 x numWaypoints vector of waypoint arrival times
%   titleSuffix    - (Optional) String suffix for titles, or cell array of comparison labels
%   lineColor      - (Optional) Line color string (single mode) or cell array {'b', 'r'} (comparison mode)
%   qddd           - (Optional) numJoints x N matrix (or {qddd1, qddd2} cell array) of joint jerks

    isComp = iscell(qd) || iscell(q) || iscell(qdd);

    if isComp
        % Comparison Mode
        if nargin >= 7 && iscell(titleSuffix)
            labels = titleSuffix;
            titleSuffix = '';
        elseif nargin >= 7 && ischar(titleSuffix) && ~isempty(titleSuffix)
            labels = {'Method 1', 'Method 2'};
            titleSuffix = ['  ', titleSuffix];
        else
            labels = {'Numerical Differentiation', 'Inverse Jacobian'};
            titleSuffix = '';
        end

        color1 = 'b';
        color2 = 'r';
        style1 = '-';
        style2 = '--';
        if nargin >= 8 && iscell(lineColor) && numel(lineColor) >= 2
            color1 = lineColor{1};
            color2 = lineColor{2};
        end

        % Extract q
        if iscell(q)
            q1 = q{1};
            q2 = q{2};
            sharedQ = false;
        else
            q1 = q;
            q2 = q;
            sharedQ = true;
        end

        % Extract qd
        qd1 = qd{1};
        qd2 = qd{2};

        % Extract qdd
        qdd1 = qdd{1};
        qdd2 = qdd{2};

        % Extract qddd
        hasJerk = (nargin >= 9 && ~isempty(qddd));
        if hasJerk
            if iscell(qddd)
                qddd1 = qddd{1};
                qddd2 = qddd{2};
            else
                qddd1 = qddd;
                qddd2 = [];
            end
        end

        numJoints = size(q1, 1);
        numWaypoints = numel(waypointTimes);

        numSubplots = 3;
        if hasJerk
            numSubplots = 4;
        end

        for i = 1:numJoints
            figure;

            % 1. Joint Position
            subplot(numSubplots, 1, 1);
            hold on;
            plot(trajTimes, q1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            if ~sharedQ
                plot(trajTimes, q2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
                legend(labels{1}, labels{2});
            else
                legend('Joint Position');
            end
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--');
            end
            if nargin >= 5 && ~isempty(jointWaypoints)
                plot(waypointTimes, jointWaypoints(i, :), 'ro', 'MarkerSize', 4);
            end
            title(['Joint ', num2str(i), titleSuffix, ' Position']);
            xlabel('Time (s)');
            ylabel('Position (rad)');
            grid on;

            % 2. Joint Velocity
            subplot(numSubplots, 1, 2);
            hold on;
            plot(trajTimes, qd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            plot(trajTimes, qd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Velocity']);
            xlabel('Time (s)');
            ylabel('Velocity (rad/s)');
            legend(labels{1}, labels{2});
            grid on;

            % 3. Joint Acceleration
            subplot(numSubplots, 1, 3);
            hold on;
            plot(trajTimes, qdd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
            plot(trajTimes, qdd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
            for wIdx = 1:numWaypoints
                xline(waypointTimes(wIdx), 'k--');
            end
            title(['Joint ', num2str(i), titleSuffix, ' Acceleration']);
            xlabel('Time (s)');
            ylabel('Acceleration (rad/s^2)');
            legend(labels{1}, labels{2});
            grid on;

            % 4. Joint Jerk (if provided)
            if hasJerk
                subplot(numSubplots, 1, 4);
                hold on;
                plot(trajTimes, qddd1(i, :), 'Color', color1, 'LineStyle', style1, 'LineWidth', 1.5);
                if ~isempty(qddd2)
                    plot(trajTimes, qddd2(i, :), 'Color', color2, 'LineStyle', style2, 'LineWidth', 1.5);
                    legend(labels{1}, labels{2});
                else
                    legend(labels{1});
                end
                for wIdx = 1:numWaypoints
                    xline(waypointTimes(wIdx), 'k--');
                end
                title(['Joint ', num2str(i), titleSuffix, ' Jerk']);
                xlabel('Time (s)');
                ylabel('Jerk (rad/s^3)');
                grid on;
            end
        end

    else
        % Single Trajectory Mode
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
end
