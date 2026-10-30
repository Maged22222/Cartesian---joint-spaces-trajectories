function PlotTrajectoriesNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold, waypoints, waypointTimes, jointWaypoints)
   
    % Initialize task-space trajectory storage
    numTimeSteps = numel(trajTimes);
    numJoints = size(q, 1);
    numWaypoints = size(waypoints, 2);
    eePositions = zeros(3, numTimeSteps); % End-effector positions (x, y, z)
    eeOrientations = zeros(3, numTimeSteps); % End-effector orientations (roll, pitch, yaw)

    % Compute forward kinematics for each time step
    for tIdx = 1:numTimeSteps
        % Get the joint configuration at the current time step
        jointConfig = q(:, tIdx)'; % Joint positions at time tIdx (row vector)

        % Compute the end-effector pose using forward kinematics
        eePose = getTransform(puma, jointConfig, eeName);

        % Extract position (x, y, z) from the transformation matrix
        eePositions(:, tIdx) = tform2trvec(eePose);

        % Extract orientation (roll, pitch, yaw) from the transformation matrix
        eeOrientations(:, tIdx) = tform2eul(eePose); % Orientation in ZYX Euler angles
    end

    % Set small angles to zero
    eePositions(abs(eePositions) < angleThreshold) = 0;
    eeOrientations(abs(eeOrientations) < angleThreshold) = 0;

    % Compute task-space velocity and acceleration (numerical differentiation)
    dt = mean(diff(trajTimes)); % Time step
    windowSize = 5; % Moving average window size
    eeVelocities = diff(eePositions, 1, 2) / dt; % Velocity (x, y, z)
    eeVelocities = movmean(eeVelocities, windowSize, 2);
    eeAngularVelocities = diff(eeOrientations, 1, 2) / dt; % Angular velocity (roll, pitch, yaw)
    eeAngularVelocities = movmean(eeAngularVelocities, windowSize, 2);

    eeAccelerations = diff(eeVelocities, 1, 2) / dt; % Acceleration (x, y, z)
    eeAccelerations = movmean(eeAccelerations, windowSize, 2);
    eeAngularAccelerations = diff(eeAngularVelocities, 1, 2) / dt; % Angular acceleration (roll, pitch, yaw)
    eeAngularAccelerations = movmean(eeAngularAccelerations, windowSize, 2);

    % Compute joint velocities and accelerations
    jointVelocities = diff(q, 1, 2) / dt;
    jointVelocities = movmean(jointVelocities, windowSize, 2);
    jointVelocities = [zeros(numJoints, 1), jointVelocities]; % Pad with zeros for the first time step
    jointAccelerations = diff(jointVelocities, 1, 2) / dt;
    jointAccelerations = movmean(jointAccelerations, windowSize, 2);
    jointAccelerations = [zeros(numJoints, 1), jointAccelerations]; % Pad with zeros for the first time step

    % Adjust time vectors for velocity and acceleration
    trajTimesVel = trajTimes(1:end-1); % Time vector for velocity
    trajTimesAcc = trajTimes(1:end-2); % Time vector for acceleration

    % Plot task-space position trajectory
    figure;
    subplot(3, 1, 1);
    plot(trajTimes, eePositions(1, :), 'LineWidth', 1.5);
    title('End-Effector X Position');
    xlabel('Time (s)');
    ylabel('X (m)');
    grid on;

    subplot(3, 1, 2);
    plot(trajTimes, eePositions(2, :), 'LineWidth', 1.5);
    title('End-Effector Y Position');
    xlabel('Time (s)');
    ylabel('Y (m)');
    grid on;

    subplot(3, 1, 3);
    plot(trajTimes, eePositions(3, :), 'LineWidth', 1.5);
    title('End-Effector Z Position');
    xlabel('Time (s)');
    ylabel('Z (m)');
    grid on;

    % Plot task-space velocity trajectory
    figure;
    subplot(3, 1, 1);
    plot(trajTimesVel, eeVelocities(1, :), 'LineWidth', 1.5);
    title('End-Effector X Velocity');
    xlabel('Time (s)');
    ylabel('Velocity X (m/s)');
    grid on;

    subplot(3, 1, 2);
    plot(trajTimesVel, eeVelocities(2, :), 'LineWidth', 1.5);
    title('End-Effector Y Velocity');
    xlabel('Time (s)');
    ylabel('Velocity Y (m/s)');
    grid on;

    subplot(3, 1, 3);
    plot(trajTimesVel, eeVelocities(3, :), 'LineWidth', 1.5);
    title('End-Effector Z Velocity');
    xlabel('Time (s)');
    ylabel('Velocity Z (m/s)');
    grid on;

    % Plot task-space acceleration trajectory
    figure;
    subplot(3, 1, 1);
    plot(trajTimesAcc, eeAccelerations(1, :), 'LineWidth', 1.5);
    title('End-Effector X Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration X (m/s^2)');
    grid on;

    subplot(3, 1, 2);
    plot(trajTimesAcc, eeAccelerations(2, :), 'LineWidth', 1.5);
    title('End-Effector Y Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration Y (m/s^2)');
    grid on;

    subplot(3, 1, 3);
    plot(trajTimesAcc, eeAccelerations(3, :), 'LineWidth', 1.5);
    title('End-Effector Z Acceleration');
    xlabel('Time (s)');
    ylabel('Acceleration Z (m/s^2)');
    grid on;

    
% Plot task-space orientation position trajectory 
figure;
subplot(3, 1, 1);
plot(trajTimes, (eeOrientations(1, :)), 'LineWidth', 1.5);
title('End-Effector Roll');
xlabel('Time (s)');
ylabel('Roll (°)');
grid on;

subplot(3, 1, 2);
plot(trajTimes,(eeOrientations(2, :)), 'LineWidth', 1.5);
title('End-Effector Pitch');
xlabel('Time (s)');
ylabel('Pitch (°)');
grid on;

subplot(3, 1, 3);
plot(trajTimes, (eeOrientations(3, :)), 'LineWidth', 1.5);
title('End-Effector Yaw');
xlabel('Time (s)');
ylabel('Yaw (°)');
grid on;

% Plot task-space  orientation velocity trajectory
figure;
subplot(3, 1, 1);
plot(trajTimesVel, eeAngularVelocities(1, :), 'LineWidth', 1.5);
title('End-Effector Roll Velocity');
xlabel('Time (s)');
ylabel(' Angular Velocity Roll (m/s)');
grid on;

subplot(3, 1, 2);
plot(trajTimesVel, eeAngularVelocities(2, :), 'LineWidth', 1.5);
title('End-Effector Pitch Velocity');
xlabel('Time (s)');
ylabel(' AngularVelocity Pitch (m/s)');
grid on;

subplot(3, 1, 3);
plot(trajTimesVel, eeAngularVelocities(3, :), 'LineWidth', 1.5);
title('End-Effector Yaw Velocity');
xlabel('Time (s)');
ylabel(' Angular Velocity Yaw (m/s)');
grid on;

% Plot task-space  orientation acceleration trajectory
figure;
subplot(3, 1, 1);
plot(trajTimesAcc, eeAngularAccelerations(1, :), 'LineWidth', 1.5);
title('End-Effector  Roll Acceleration');
xlabel('Time (s)');
ylabel('Angular Acceleration  Roll (m/s^2)');
grid on;

subplot(3, 1, 2);
plot(trajTimesAcc, eeAngularAccelerations(2, :), 'LineWidth', 1.5);
title('End-Effector Pitch Acceleration');
xlabel('Time (s)');
ylabel('Angular Acceleration Pitch (m/s^2)');
grid on;

subplot(3, 1, 3);
plot(trajTimesAcc, eeAngularAccelerations(3, :), 'LineWidth', 1.5);
title('End-Effector Yaw Acceleration ');
xlabel('Time (s)');
ylabel('Angular Acceleration Yaw (m/s^2)');
grid on;


    % 3D Plot of the end-effector trajectory in task space
    figure;
    plot3(eePositions(1, :), eePositions(2, :), eePositions(3, :), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'MarkerSize', 6); % Mark waypoints
    title('End-Effector Task-Space Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    legend('Trajectory', 'Waypoints');

    % Plot joint-space results
    for i = 1:numJoints
        figure;

        % Plot joint position
        subplot(3, 1, 1);
        hold on;
        plot(trajTimes, q(i, :), 'LineWidth', 1.5);
        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--'); % Mark waypoint times
        end
       % plot(waypointTimes, jointWaypoints(i, :), 'ro', 'MarkerSize', 2); % Mark waypoints
        title(['Joint ', num2str(i), ' Position']);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        legend('Trajectory', 'Waypoints');
        grid on;

        % Plot joint velocity
        subplot(3, 1, 2);
        hold on;
        plot(trajTimes, jointVelocities(i, :), 'LineWidth', 1.5);
        title(['Joint ', num2str(i), ' Velocity']);
        xlabel('Time (s)');
        ylabel('Velocity (rad/s)');
        grid on;

        % Plot joint acceleration
        subplot(3, 1, 3);
        hold on;
        plot(trajTimes, jointAccelerations(i, :), 'LineWidth', 1.5);
        title(['Joint ', num2str(i), ' Acceleration']);
        xlabel('Time (s)');
        ylabel('Acceleration (rad/s^2)');
        grid on;
    end
 
end