function plotTaskSpaceTrajectories(trajTimes, taskPos, taskVel, taskAcc, taskOrientPos, taskOrientVel, taskOrientAcc)
% PLOTTASKSPACETRAJECTORIES Plots task-space position, velocity, acceleration,
% and optionally task-space orientation profiles (position, velocity, acceleration).
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

    % Plot Orientation (Roll, Pitch, Yaw) if provided
    hasOrient = (nargin >= 5 && ~isempty(taskOrientPos));
    if hasOrient
        figure;

        % Orientation Position
        subplot(3, 1, 1);
        plot(trajTimes, taskOrientPos(1, :), 'r', 'LineWidth', 1.5); hold on;
        plot(trajTimes, taskOrientPos(2, :), 'g', 'LineWidth', 1.5);
        plot(trajTimes, taskOrientPos(3, :), 'b', 'LineWidth', 1.5);
        title('Orientation (Roll, Pitch, Yaw)');
        xlabel('Time (s)');
        ylabel('Orientation (rad)');
        legend('Roll', 'Pitch', 'Yaw');
        grid on;

        % Orientation Velocity
        subplot(3, 1, 2);
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
        subplot(3, 1, 3);
        if nargin >= 7 && ~isempty(taskOrientAcc)
            plot(trajTimes, taskOrientAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
            plot(trajTimes, taskOrientAcc(2, :), 'g', 'LineWidth', 1.5);
            plot(trajTimes, taskOrientAcc(3, :), 'b', 'LineWidth', 1.5);
        end
        title('Orientation Acceleration (Roll, Pitch, Yaw)');
        xlabel('Time (s)');
        ylabel('Angular Acceleration (rad/s^2)');
        legend('Roll', 'Pitch', 'Yaw');
        grid on;
    end
end
