function plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints, dt, waypoints, title3D)
% PLOTS Plots task-space position, orientation, velocity, acceleration, jerks,
% and optionally the 3D task-space trajectory. Includes (includeOrientation = true/false)
% in the title of every figure.
%
% Syntax:
%   plots(trajTimes, eePos, eeOrient, eeVel, eeAcc)
%   plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd)
%   plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints)
%   plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints, dt)
%   plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints, dt, waypoints, title3D)
%
% Inputs:
%   trajTimes - 1 x N vector of trajectory time steps
%   eePos     - 3 x N matrix of end-effector positions [x; y; z]
%   eeOrient  - 3 x N matrix of end-effector orientations [roll; pitch; yaw]
%   eeVel     - 6 x N (or 6 x (N-1)) matrix of task velocities (1:3 angular, 4:6 linear)
%   eeAcc     - 6 x N (or 6 x (N-2)) matrix of task accelerations (1:3 angular, 4:6 linear)
%   qdd       - (Optional) numJoints x N matrix of joint accelerations (kept for signature compatibility)
%   numJoints - (Optional) Number of joints
%  

    numSteps = numel(trajTimes);

    if nargin < 8 || isempty(dt)
        if numSteps > 1
            dt = mean(diff(trajTimes));
        else
            dt = 0.02;
        end
    end

    % Determine includeOrientation tag for all figure titles
    orientTag = '';
    if nargin >= 10 && ~isempty(title3D)
        if islogical(title3D)
            if title3D
                orientTag = ' (includeOrientation = true)';
            else
                orientTag = ' (includeOrientation = false)';
            end
        elseif ischar(title3D) || isstring(title3D)
            strTitle = char(title3D);
            if contains(strTitle, 'includeOrientation = true', 'IgnoreCase', true)
                orientTag = ' (includeOrientation = true)';
            elseif contains(strTitle, 'includeOrientation = false', 'IgnoreCase', true)
                orientTag = ' (includeOrientation = false)';
            elseif ~startsWith(strTitle, ' ')
                orientTag = [' (', strTitle, ')'];
            else
                orientTag = strTitle;
            end
        end
    end

    % Fallback: automatically check caller workspace for includeOrientation variable
    if isempty(orientTag)
        try
            incOrient = evalin('caller', 'includeOrientation');
            if islogical(incOrient) || isnumeric(incOrient)
                if incOrient
                    orientTag = ' (includeOrientation = true)';
                else
                    orientTag = ' (includeOrientation = false)';
                end
            end
        catch
            orientTag = '';
        end
    end

    % Adjust time vectors to match data dimensions (handles numerical diff lengths N-1, N-2)
    tPos = trajTimes(1:size(eePos, 2));
    tOrient = trajTimes(1:size(eeOrient, 2));
    tVel = trajTimes(1:size(eeVel, 2));
    tAcc = trajTimes(1:size(eeAcc, 2));

    % Compute Task-Space Jerk (time derivative of task-space acceleration eeAcc)
    if size(eeAcc, 2) > 1
        eeJerk = diff(eeAcc, 1, 2) / dt;
        tJerk = trajTimes(1:size(eeJerk, 2));
    else
        eeJerk = zeros(size(eeAcc));
        tJerk = tAcc;
    end

    % Figure 1: Task-Space Position, Orientation, and Velocity
    figure;

    % Position
    subplot(3, 1, 1);
    plot(tPos, eePos(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tPos, eePos(2, :), 'g', 'LineWidth', 1.5);
    plot(tPos, eePos(3, :), 'b', 'LineWidth', 1.5);
    title(['Task-Space Position', orientTag]);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X', 'Y', 'Z');
    grid on;

    % Plot orientation (roll, pitch, yaw)
    subplot(3, 1, 2);
    plot(tOrient, eeOrient(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tOrient, eeOrient(2, :), 'g', 'LineWidth', 1.5);
    plot(tOrient, eeOrient(3, :), 'b', 'LineWidth', 1.5);
    title(['Task-Space Orientation (Roll, Pitch, Yaw)', orientTag]);
    xlabel('Time (s)');
    ylabel('Orientation (rad)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;

    % Plot velocity (linear + angular)
    subplot(3, 1, 3);
    plot(tVel, eeVel(4, :), 'r', 'LineWidth', 1.5); hold on; % Linear X
    plot(tVel, eeVel(5, :), 'g', 'LineWidth', 1.5); % Linear Y
    plot(tVel, eeVel(6, :), 'b', 'LineWidth', 1.5); % Linear Z
    plot(tVel, eeVel(1, :), 'c--', 'LineWidth', 1.5); % Angular X
    plot(tVel, eeVel(2, :), 'm--', 'LineWidth', 1.5); % Angular Y
    plot(tVel, eeVel(3, :), 'k--', 'LineWidth', 1.5); % Angular Z
    title(['Task-Space Velocity (Linear + Angular)', orientTag]);
    xlabel('Time (s)');
    ylabel('Velocity');
    legend('Vx', 'Vy', 'Vz', 'Wx', 'Wy', 'Wz');
    grid on;

    % Figure 2: Task-Space Acceleration (Linear + Angular)
    figure;
    plot(tAcc, eeAcc(4, :), 'r', 'LineWidth', 1.5); hold on; % Linear X
    plot(tAcc, eeAcc(5, :), 'g', 'LineWidth', 1.5); % Linear Y
    plot(tAcc, eeAcc(6, :), 'b', 'LineWidth', 1.5); % Linear Z
    plot(tAcc, eeAcc(1, :), 'c--', 'LineWidth', 1.5); % Angular X
    plot(tAcc, eeAcc(2, :), 'm--', 'LineWidth', 1.5); % Angular Y
    plot(tAcc, eeAcc(3, :), 'k--', 'LineWidth', 1.5); % Angular Z
    title(['Task-Space Acceleration (Linear + Angular)', orientTag]);
    xlabel('Time (s)');
    ylabel('Acceleration');
    legend('Ax', 'Ay', 'Az', 'AlphaX', 'AlphaY', 'AlphaZ');
    grid on;

    % Figure 3: Task-Space Jerks (Linear + Angular)
    figure;
    plot(tJerk, eeJerk(4, :), 'r', 'LineWidth', 1.5); hold on; % Linear X Jerk
    plot(tJerk, eeJerk(5, :), 'g', 'LineWidth', 1.5); % Linear Y Jerk
    plot(tJerk, eeJerk(6, :), 'b', 'LineWidth', 1.5); % Linear Z Jerk
    plot(tJerk, eeJerk(1, :), 'c--', 'LineWidth', 1.5); % Angular X Jerk
    plot(tJerk, eeJerk(2, :), 'm--', 'LineWidth', 1.5); % Angular Y Jerk
    plot(tJerk, eeJerk(3, :), 'k--', 'LineWidth', 1.5); % Angular Z Jerk
    title(['Task-Space Jerk (Linear + Angular)', orientTag]);
    xlabel('Time (s)');
    ylabel('Jerk');
    legend('Jx', 'Jy', 'Jz', 'AlphaDotX', 'AlphaDotY', 'AlphaDotZ');
    grid on;

    % Optional Figure 4: 3D Task-Space Trajectory
    if nargin >= 9 && ~isempty(waypoints)
        figure;
        plot3(eePos(1, :), eePos(2, :), eePos(3, :), 'b-', 'LineWidth', 1.5);
        hold on;
        plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'MarkerSize', 6);
        if nargin >= 10 && ~isempty(title3D) && (ischar(title3D) || isstring(title3D))
            title(char(title3D));
        else
            title(['End-Effector Task-Space Trajectory', orientTag]);
        end
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        grid on;
        legend('Trajectory', 'Waypoints');
    end
end
