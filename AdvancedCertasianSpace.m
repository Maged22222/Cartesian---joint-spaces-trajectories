%% Setup
clear, clc, close all

% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 0 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypoints(:,4)=waypoints(:,1);
waypointTimes = [0 1 2 3]; % Time at each waypoint
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
trajType = 'quintic';
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
         jointWaypoints(:,idx)= config_all(idx,:)';
         
    end
    % Show the robot
    show(puma,config_all(idx,:),'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end

q=config_all';
%% plot results
if ikWeights == [1 1 1 1 1 1]
      computeAndPlotTrajectorieswWithJacobianInCartisainSpace(puma, trajTimes, config_all, taskPos, taskVel, taskAcc, taskOrientVel, taskOrientAcc, eeName, angleThreshold,waypoints)
else
      PlotTrajectoriesNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold, waypoints, waypointTimes, jointWaypoints)
end