function computeAndPlotTrajectorieswWithJacobianInCartisainSpace(puma, trajTimes, config_all, taskPos, taskVel, taskAcc, taskOrientVel, taskOrientAcc, eeName, angleThreshold,waypoints)


    % Initialize variables
    numSteps = numel(trajTimes);
    numJoints = size(config_all, 2);
    q = config_all'; % Joint positions (numJoints x numSteps)
    qd = zeros(numJoints, numSteps); % Joint velocities
    qdd = zeros(numJoints, numSteps); % Joint accelerations

    % Compute joint velocities and accelerations using the inverse Jacobian
    for idx = 1:numSteps
        % Get joint configuration at this time step
        config = q(:, idx)';

        % Compute Jacobian at the current configuration
        J = geometricJacobian(puma, config, eeName);

        % Check for singularity
        % Extract task-space velocity and acceleration at this time step
        eeVel = [taskOrientVel(:, idx); taskVel(:, idx)]; % Angular + Linear velocity
        eeAcc = [taskOrientAcc(:, idx); taskAcc(:, idx)]; % Angular + Linear acceleration

        % SVD-based Adaptive Damped Least Squares (DLS)
        lambda = 0.05;
        epsilon = 0.005;
        [U, S, V] = svd(J, 'econ');
        s = diag(S);
        sigma_min = min(s);
        if sigma_min < epsilon
            lam2 = lambda^2 * (1 - (sigma_min / epsilon)^2);
        else
            lam2 = 0; % Exact un-damped inversion
        end
        s_dls = s ./ (s.^2 + lam2);
        J_inv = V * diag(s_dls) * U';

        % Compute joint velocities using the DLS inverse Jacobian
        qd(:, idx) = J_inv * eeVel;

        % Time derivative of the Jacobian along the motion: dJ/dt = sum_i (dJ/dq_i) * qd_i
        % Computed as a central directional derivative in the direction of qd
        % (exact up to O(h^2), valid for all time steps including idx = 1, independent of dt).
        h = 1e-6;
        currentJointVel = qd(:, idx)';
        Jplus  = geometricJacobian(puma, config + h * currentJointVel, eeName);
        Jminus = geometricJacobian(puma, config - h * currentJointVel, eeName);
        Jdot = (Jplus - Jminus) / (2 * h);

        % Compute joint accelerations using inverse Jacobian relation:
        % eeAcc = J*qdd + Jdot*qd  =>  qdd = J_inv * (eeAcc - Jdot * qd(:, idx))
        qdd(:, idx) = J_inv * (eeAcc - Jdot * qd(:, idx));
    end

    % Plot Joint-Space Trajectory
    figure;
    plot(trajTimes, q', 'LineWidth', 1.5);
    title('Joint Positions');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes, qd', 'LineWidth', 1.5);
    title('Joint Velocities');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    figure;
    plot(trajTimes, qdd', 'LineWidth', 1.5);
    title('Joint Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
    grid on;

    % Plot Task-Space Trajectory (Position, Velocity, Acceleration)
    figure;
    plot(trajTimes, taskPos(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskPos(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskPos(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Position (x, y, z)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y', 'z');
    grid on;

    figure;
    plot(trajTimes, taskVel(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskVel(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskVel(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Velocity (x, y, z)');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('x', 'y', 'z');
    grid on;

    figure;
    plot(trajTimes, taskAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskAcc(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskAcc(3, :), 'b', 'LineWidth', 1.5);
    title('Task-Space Acceleration (x, y, z)');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend('x', 'y', 'z');
    grid on;

    % Plot Orientation (Roll, Pitch, Yaw)
    figure;
    subplot(3, 1, 1);
    plot(trajTimes, taskOrientVel(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskOrientVel(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskOrientVel(3, :), 'b', 'LineWidth', 1.5);
    title('Orientation Velocity (Roll, Pitch, Yaw)');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;

    subplot(3, 1, 2);
    plot(trajTimes, taskOrientAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskOrientAcc(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskOrientAcc(3, :), 'b', 'LineWidth', 1.5);
    title('Orientation Acceleration (Roll, Pitch, Yaw)');
    xlabel('Time (s)');
    ylabel('Angular Acceleration (rad/s^2)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;

    % Orientation Acceleration
    subplot(3, 1, 3);
    plot(trajTimes, taskOrientAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(trajTimes, taskOrientAcc(2, :), 'g', 'LineWidth', 1.5);
    plot(trajTimes, taskOrientAcc(3, :), 'b', 'LineWidth', 1.5);
    title('Orientation Acceleration (Roll, Pitch, Yaw)');
    xlabel('Time (s)');
    ylabel('Angular Acceleration (rad/s^2)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;

     % 3D Plot of the end-effector trajectory in task space
    figure;
    plot3(taskPos(1, :),  taskPos(2, :), taskPos(3, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'MarkerSize', 6); % Mark waypoints
    title('End-Effector Task-Space Trajectory (includeOrientation = false)');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    legend('Trajectory', 'Waypoints');
end