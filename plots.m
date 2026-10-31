function plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints, dt, waypoints, title3D)
% PLOTS Plots task-space position, orientation, velocity, acceleration, joint jerks,
% and optionally the 3D task-space trajectory.
%
% Syntax:
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
%   qdd       - numJoints x N (or N x numJoints) matrix of joint accelerations
%   numJoints - (Optional) Number of joints (default: min(size(qdd)))
%   dt        - (Optional) Time step for jerk differentiation (default: 0.02 or mean(diff(trajTimes)))
%   waypoints - (Optional) 3 x M matrix of waypoints to plot 3D trajectory
%   title3D   - (Optional) Title for the 3D trajectory plot

    numSteps = numel(trajTimes);

    if nargin < 7 || isempty(numJoints)
        numJoints = min(size(qdd));
    end

    if nargin < 8 || isempty(dt)
        if numSteps > 1
            dt = mean(diff(trajTimes));
        else
            dt = 0.02;
        end
    end

    % Adjust time vectors to match data dimensions (handles numerical diff lengths N-1, N-2)
    tPos = trajTimes(1:size(eePos, 2));
    tOrient = trajTimes(1:size(eeOrient, 2));
    tVel = trajTimes(1:size(eeVel, 2));
    tAcc = trajTimes(1:size(eeAcc, 2));

    % Ensure qdd is formatted as (numSteps x numJoints) for jerk computation
    if size(qdd, 1) == numJoints && size(qdd, 2) == numSteps
        qdd_matrix = qdd';
    else
        qdd_matrix = qdd;
    end

    % Figure 1: Task-Space Position, Orientation, and Velocity
    figure;

    % Position
    subplot(3, 1, 1);
    plot(tPos, eePos(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tPos, eePos(2, :), 'g', 'LineWidth', 1.5);
    plot(tPos, eePos(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Position');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X', 'Y', 'Z');
    grid on;

    % Plot orientation (roll, pitch, yaw)
    subplot(3, 1, 2);
    plot(tOrient, eeOrient(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tOrient, eeOrient(2, :), 'g', 'LineWidth', 1.5);
    plot(tOrient, eeOrient(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Orientation (Roll, Pitch, Yaw)');
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
    title('Task-Space Velocity (Linear + Angular)');
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
    title('Task-Space Acceleration (Linear + Angular)');
    xlabel('Time (s)');
    ylabel('Acceleration');
    legend('Ax', 'Ay', 'Az', 'AlphaX', 'AlphaY', 'AlphaZ');
    grid on;

    % Compute and plot joint-space jerks
    jointJerks = zeros(numSteps, numJoints);
    for tIdx = 1:numSteps
        if tIdx > 1
            jointJerks(tIdx, :) = (qdd_matrix(tIdx, :) - qdd_matrix(tIdx - 1, :)) / dt;
        end
    end

    timeVectorJerk = trajTimes(3:end);

    % Figure 3: Joint-Space Jerks
    figure;
    plot(timeVectorJerk, jointJerks(3:end, :), 'LineWidth', 1.5);
    title('Joint-Space Jerks');
    xlabel('Time (s)');
    ylabel('Jerk (rad/s^3)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    % Optional Figure 4: 3D Task-Space Trajectory
    if nargin >= 9 && ~isempty(waypoints)
        figure;
        plot3(eePos(1, :), eePos(2, :), eePos(3, :), 'b-', 'LineWidth', 1.5);
        hold on;
        plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'MarkerSize', 6);
        if nargin >= 10 && ~isempty(title3D)
            title(title3D);
        else
            title('End-Effector Task-Space Trajectory');
        end
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        grid on;
        legend('Trajectory', 'Waypoints');
    end
end
