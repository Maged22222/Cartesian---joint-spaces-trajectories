%% Setup
clear, clc, close all

% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 0 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypointTimes = [0 2 4 6]; % Time at each waypoint
waypointAccelTimes = [0.5 0.5 0.5]; % Acceleration time for each segment
numWaypoints = size(waypoints,2);
% Use a small sample time for this example
ts = 0.02;
trajTimes = 0:ts:waypointTimes(end);

taskOrientWaypoints =zeros(3,numel(trajTimes));
taskOrientVel = zeros(3, numel(trajTimes)); % Angular velocity for all time steps
taskOrientAcc = zeros(3, numel(trajTimes)); % Angular acceleration for all time

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Define threshold for small angles
angleThreshold = 0.000001;
% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikWeights = [0 0 0 1 1 1];
ikInitGuess =jointAnglesHome;
numJoints = numel(puma.homeConfiguration);

% Set up plot
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Changing Joint Limits
for idx = 1:numJoints
 
  puma.Bodies{idx}.Joint.PositionLimits = [-pi, pi];
end

%% Generate trajectory
% Cartesian Motion only
trajType =  'quintic';
switch trajType
    case 'trap'
        [taskPos,taskVel,taskAcc] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
                            
    case 'cubic'
        [taskPos,taskVel,taskAcc] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(3,numWaypoints));
        
    case 'quintic'
        [taskPos,taskVel,taskAcc] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(3,numWaypoints), ...
            'AccelerationBoundaryCondition',zeros(3,numWaypoints));
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [taskPos,taskVel,taskAcc] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end

% Show the full trajectory with the rigid body tree
if plotMode == 1
    set(hTraj,'xdata',taskPos(1,:),'ydata',taskPos(2,:),'zdata',taskPos(3,:));
    TrajPoints = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'r.-');
elseif plotMode == 2
    plotTransforms(taskPos',repmat([1 0 0 0],[size(taskPos,2) 1]),'FrameSize',0.05);
end

 config_all=zeros(numel(trajTimes),6);
%% Trajectory following loop
for idx = 1:numel(trajTimes) 
    % Solve IK
    tgtPose = trvec2tform(taskPos(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);

    % Check IK solver status
     if strcmp(info.Status, 'success')
          ikInitGuess = config;
           % Set small angles to zero
          config(abs(config) < angleThreshold) = 0;
          config_all(idx,:)=config;
    else
        warning('IK solver failed at time step %d', idx);

     end
    

       eePos = taskPos(:,idx)';
       set(TrajPoints,'xdata',[TrajPoints.XData eePos(1)], ...
                  'ydata',[TrajPoints.YData eePos(2)], ...
                  'zdata',[TrajPoints.ZData eePos(3)]);

       set(TrajPoints, 'Color', 'r'); % Set the trajectory color to black

       % Define a small tolerance for comparison
        tolerance = 1e-3; % Adjust this value as needed

     % Check if eePos is close to any waypoint
       isMatch = any(all(abs(waypoints' - taskPos(:,idx)') < tolerance, 2));

    if isMatch
         % Find Cartesian points for visualization
         eeTform = getTransform(puma,config,eeName);
         plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.1);
         
    end
    % Show the robot
    show(puma,config_all(idx,:),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end




numSteps = numel(trajTimes);
q =  config_all';    % Joint positions
qd = zeros(numJoints, numSteps);   % Joint velocities
qdd = zeros(numJoints, numSteps);  % Joint accelerations
numJoints = size(q, 1);
 % Compute joint velocities and accelerations
    jointVelocities = diff(q, 1, 2) / ts;
    %jointVelocities = movmean(jointVelocities, windowSize, 2);
    jointVelocities = [zeros(numJoints, 1), jointVelocities]; % Pad with zeros for the first time step

    jointAccelerations = diff(jointVelocities, 1, 2) / ts;
    %jointAccelerations = movmean(jointAccelerations, windowSize, 2);
    jointAccelerations = [zeros(numJoints, 1), jointAccelerations]; % Pad with zeros for the first time step

    jointJerks=diff( jointAccelerations, 1, 2) / ts;
    %jointJerks = movmean( jointJerks, windowSize, 2);
    jointJerks = [zeros(numJoints, 1),  jointJerks]; % Pad with zeros for the first time step

        % Joint-Space Plots
figure;
plot(trajTimes, q(:,1:numel(trajTimes)), 'LineWidth', 1.5);
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
grid on;

figure;
plot(trajTimes, jointVelocities(:,1:numel(trajTimes)), 'LineWidth', 1.5);
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
grid on;

figure;
plot(trajTimes, jointAccelerations(:,1:numel(trajTimes)), 'LineWidth', 1.5);
title('Joint Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
grid on;

% Adjust time vector for jerk (exclude the first two time steps)
timeVectorJerk =trajTimes(1:end);

% Plot joint-space jerk
figure;
plot(timeVectorJerk, jointJerks(:,1:numel(trajTimes)), 'LineWidth', 1.5);
title('Joint-Space Jerks');
xlabel('Time (s)');
ylabel('Jerk (rad/s^3)');
legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
grid on;
%% Compute Joint Velocities and Accelerations Using Inverse Jacobian and check the sigulirty
for idx = 1:numSteps
  % Get joint configuration at this time step
  config = q(:, idx)';

  % Compute Jacobian at the current configuration
  J = geometricJacobian(puma, config, eeName);
 
  if cond(J) > 1e6
  warning('Jacobian is near-singular at time step %d', idx);
  qd(:, idx) = zeros(numJoints, 1);
  qdd(:, idx) = zeros(numJoints, 1);
  continue;
  end
 % Extract task-space velocity and acceleration at this time step
  eeVel = [taskOrientVel(:, idx); taskVel(:, idx)]; % Angular (orientation) + Linear (position) velocity
  eeAcc = [taskOrientAcc(:, idx); taskAcc(:, idx)]; % Angular (orientation) + Linear (position) acceleration
  

  % Compute joint velocities using the inverse Jacobian
  qd(:, idx) = pinv(J) * eeVel; % Use pseudo-inverse in case J is not square

  % Compute joint accelerations using the inverse Jacobian
  if idx > 1
      % Approximate dJ/dt using finite differences
      J_prev = geometricJacobian(puma, q(:, idx-1)', eeName);
      dJdt = (J - J_prev) / (trajTimes(idx) - trajTimes(idx-1));

      % Compute joint accelerations
      qdd(:, idx) = pinv(J) * eeAcc - pinv(J) * dJdt * qd(:, idx-1);
  else
      % For the first time step, assume zero acceleration
      qdd(:, idx) = zeros(numJoints, 1);
  end
end

%% jerks
qdd=qdd';
jointJerks = zeros(numel(trajTimes), numJoints); % Initialize joint jerks
for tIdx = 1:numel(trajTimes)
            if tIdx>1  % Compute joint jerks
                jointJerks(tIdx, :) = (qdd(tIdx, :) - qdd(tIdx - 1, :)) /0.02;
            end
end

% Adjust time vector for jerk (exclude the first two time steps)
timeVectorJerk = trajTimes(3:end);

% Plot joint-space jerk
figure;
plot(timeVectorJerk, jointJerks(3:end, :), 'LineWidth', 1.5);
title('Joint-Space Jerks');
xlabel('Time (s)');
ylabel('Jerk (rad/s^3)');
legend(arrayfun(@(x) sprintf('Joint %d', x), 1:numJoints, 'UniformOutput', false));
grid on;
%% Plot Joint-Space Trajectory
figure;

% Plot joint positions
figure;
plot(trajTimes, q', 'LineWidth', 1.5);
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

% Plot joint velocities
figure;
plot(trajTimes, qd', 'LineWidth', 1.5);
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

% Plot joint accelerations
figure;
plot(trajTimes, qdd', 'LineWidth', 1.5);
title('Joint Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;


%% Plot Task-Space Trajectory (Position, Velocity, Acceleration)
figure;

% Position
figure;
plot(trajTimes, taskPos(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(trajTimes, taskPos(2, :), 'g', 'LineWidth', 1.5);
plot(trajTimes, taskPos(3, :), 'b', 'LineWidth', 1.5);
title('Task-Space Position (x, y, z)');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');
grid on;

% Velocity
figure;
plot(trajTimes, taskVel(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(trajTimes, taskVel(2, :), 'g', 'LineWidth', 1.5);
plot(trajTimes, taskVel(3, :), 'b', 'LineWidth', 1.5);
title('Task-Space Velocity (x, y, z)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('x', 'y', 'z');
grid on;

% Acceleration
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

% Orientation Position
subplot(3, 1, 1);
plot(trajTimes, taskOrientWaypoints(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(trajTimes, taskOrientWaypoints(2, :), 'g', 'LineWidth', 1.5);
plot(trajTimes, taskOrientWaypoints(3, :), 'b', 'LineWidth', 1.5);
title('Orientation (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Orientation (rad)');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% Orientation Velocity
subplot(3, 1, 2);
plot(trajTimes, taskOrientVel(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(trajTimes, taskOrientVel(2, :), 'g', 'LineWidth', 1.5);
plot(trajTimes, taskOrientVel(3, :), 'b', 'LineWidth', 1.5);
title('Orientation Velocity (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
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