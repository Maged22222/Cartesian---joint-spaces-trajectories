function plotTaskSpaceTrajectories(trajTimes, taskPos, taskVel, taskAcc, taskOrientPos, taskOrientVel, taskOrientAcc)
% PLOTTASKSPACETRAJECTORIES Plots task-space position, velocity, acceleration, jerks,
% and optionally task-space orientation profiles (position, velocity, acceleration, jerk).
%
% Syntax:
%   plotTaskSpaceTrajectories(trajTimes, taskPos, taskVel, taskAcc)
%   plotTaskSpaceTrajectories(trajTimes, taskPos, taskVel, taskAcc, taskOrientPos, taskOrientVel, taskOrientAcc)
%
% Inputs:
%   trajTimes     - 1 x N vector of trajectory timestamps
%   taskPos       - 3 x N matrix of end-effector positions [x; y; z]
%   taskVel       - 3 x N matrix of end-effector linear velocities [vx; vy; vz]
%   taskAcc       - 3 x N matrix of end-effector linear accelerations [ax; ay; az]
%   taskOrientPos - (Optional) 3 x N matrix of orientation angles [roll; pitch; yaw]
%   taskOrientVel - (Optional) 3 x N matrix of angular velocities [wx; wy; wz]
%   taskOrientAcc - (Optional) 3 x N matrix of angular accelerations [alphax; alphay; alphaz]

    dt = mean(diff(trajTimes));

    % Task-Space Position
    figure;
    plot(trajTimes, taskPos(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskPos(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskPos(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Position (x, y, z)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y', 'z');
    grid on;

    % Task-Space Velocity
    figure;
    plot(trajTimes, taskVel(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskVel(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskVel(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Velocity (x, y, z)');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('x', 'y', 'z');
    grid on;

    % Task-Space Acceleration
    figure;
    plot(trajTimes, taskAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskAcc(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskAcc(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Acceleration (x, y, z)');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('x', 'y', 'z');
    grid on;

    % Task-Space Jerk (Linear)
    if size(taskAcc, 2) > 1
        taskJerk = diff(taskAcc, 1, 2) / dt;
        tJerk = trajTimes(1:size(taskJerk, 2));
    else
        taskJerk = zeros(size(taskAcc));
        tJerk = trajTimes;
    end

    figure;
    plot(tJerk, taskJerk(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tJerk, taskJerk(2, :), 'g', 'LineWidth', 1.5);
    plot(tJerk, taskJerk(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Jerk (x, y, z)');
    xlabel('Time (s)');
    ylabel('Jerk (m/s^3)');
    legend('Jx', 'Jy', 'Jz');
    grid on;

    % Plot Orientation (Roll, Pitch, Yaw) if provided
    activateorentation = (nargin >= 5 && ~isempty(taskOrientPos));
    if activateorentation
        figure;

        % Check if angular acceleration is provided to compute angular jerk
        hasOrientAcc = (nargin >= 7 && ~isempty(taskOrientAcc));
        numSubplots = 3;
        if hasOrientAcc
            numSubplots = 4;
        end

        % Orientation Position
        subplot(numSubplots, 1, 1);
        plot(trajTimes, taskOrientPos(1, :), 'r', 'LineWidth', 1.5); hold on;
        plot(trajTimes, taskOrientPos(2, :), 'g', 'LineWidth', 1.5);
        plot(trajTimes, taskOrientPos(3, :), 'b', 'LineWidth', 1.5);
        title('Orientation (Roll, Pitch, Yaw)');
        xlabel('Time (s)');
        ylabel('Orientation (rad)');
        legend('Roll', 'Pitch', 'Yaw');
        grid on;

        % Orientation Velocity
        subplot(numSubplots, 1, 2);
        if nargin >= 6 && ~isempty(taskOrientVel)
            plot(trajTimes, taskOrientVel(1, :), 'r', 'LineWidth', 1.5); hold on;
            plot(trajTimes, taskOrientVel(2, :), 'g', 'LineWidth', 1.5);
            plot(trajTimes, taskOrientVel(3, :), 'b', 'LineWidth', 1.5);
        end
        title('Orientation Velocity (Roll, Pitch, Yaw)');
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        legend('Roll', 'Pitch', 'Yaw');
        grid on;

        % Orientation Acceleration
        subplot(numSubplots, 1, 3);
        if hasOrientAcc
            plot(trajTimes, taskOrientAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
            plot(trajTimes, taskOrientAcc(2, :), 'g', 'LineWidth', 1.5);
            plot(trajTimes, taskOrientAcc(3, :), 'b', 'LineWidth', 1.5);
        end
        title('Orientation Acceleration (Roll, Pitch, Yaw)');
        xlabel('Time (s)');
        ylabel('Angular Acceleration (rad/s^2)');
        legend('Roll', 'Pitch', 'Yaw');
        grid on;

        % Orientation Jerk
        if hasOrientAcc
            if size(taskOrientAcc, 2) > 1
                taskOrientJerk = diff(taskOrientAcc, 1, 2) / dt;
                tOrientJerk = trajTimes(1:size(taskOrientJerk, 2));
            else
                taskOrientJerk = zeros(size(taskOrientAcc));
                tOrientJerk = trajTimes;
            end

            subplot(numSubplots, 1, 4);
            plot(tOrientJerk, taskOrientJerk(1, :), 'r', 'LineWidth', 1.5); hold on;
            plot(tOrientJerk, taskOrientJerk(2, :), 'g', 'LineWidth', 1.5);
            plot(tOrientJerk, taskOrientJerk(3, :), 'b', 'LineWidth', 1.5);
            title('Orientation Jerk (Roll, Pitch, Yaw)');
            xlabel('Time (s)');
            ylabel('Angular Jerk (rad/s^3)');
            legend('Roll Jerk', 'Pitch Jerk', 'Yaw Jerk');
            grid on;
        end
    end
end
