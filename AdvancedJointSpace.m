%% Setup
clear, clc, close all
%trajType = input('Enter trajectory type (trap, cubic, quintic, bspline): ', 's');
% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 0 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypoints(:,4)=waypoints(:,1);
orientations = [0 0 0; 0 pi/4 0; 0 pi/2 0; 0 pi 0];
waypointTimes = [0 2 4 6]; % Time at each waypoint
waypointAccelTimes = [0.5 0.5 0.5]; % Acceleration time for each segment
trajType = 'quintic';
% Define nonzero velocity boundary conditions for all joints
velocityBoundaryCondition = [0 0.5 0.5 0;  % Joint 1 velocities
                            0 1.0 1.0 0;  % Joint 2 velocities
                            0 1.5 1.5 0;  % Joint 3 velocities
                            0 0.2 0.2 0;  % Joint 4 velocities
                            0 0.1 0.1 0;  % Joint 5 velocities
                            0 0 0 0];     % Joint 6 velocities

% Define nonzero acceleration boundary conditions for all joints
accelerationBoundaryCondition = [0 0.2 0.2 0;  % Joint 1 accelerations
                                0 0.4 0.4 0;  % Joint 2 accelerations
                                0 0.6 0.6 0;  % Joint 3 accelerations
                                0 0.1 0.1 0;  % Joint 4 accelerations
                                0 0.05 0.05 0; % Joint 5 accelerations
                                0 0 0 0];     % Joint 6 accelerations

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Use a small sample time for this example
ts = 0.02;
trajTimes = 0:ts:waypointTimes(end);

% Define threshold for small angles
angleThreshold = 0.0001;


% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikWeights = [0 0 0 1 1 1];
ikInitGuess = jointAnglesHome;
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

% Set up plot
plotMode = 1; 
show(puma,puma.homeConfiguration,'Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Solve IK for all waypoints and includeOrientation = false
includeOrientation = false; % Set this to use zero vs. nonzero orientations

numWaypoints = size(waypoints,2);
numJoints = numel(puma.homeConfiguration);
jointWaypoints = zeros(numJoints,numWaypoints);

for idx = 1:numWaypoints
    if includeOrientation
        tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(idx,:));
    else
        tgtPose =  trvec2tform(waypoints(:,idx)');
    end
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess=config;
    jointWaypoints(:,idx) = config';
    
end

%% Generate trajectory on joint space
  [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
            'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
% Set small angles to zero
  q(abs(q) < angleThreshold) = 0;
%% Trajectory following loop
for idx = 1:numel(trajTimes) 

 
    config = q(:,idx)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(puma,config,eeName);
    eePos = tform2trvec(eeTform);
    % Define a small tolerance for comparison
     tolerance = 1e-3; % Adjust this value as needed

    % Check if eePos is close to any waypoint
     isMatch = any(all(abs(waypoints' - eePos) < tolerance, 2));
    if isMatch
         plotMode = 2;
         
    end
    if plotMode == 1
        
        set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                  'ydata',[hTraj.YData eePos(2)], ...
                  'zdata',[hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.1);
        plotMode = 1;
    end
    
    % Show the robot
    show(puma,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow   
    
end

%% ploting results

if ikWeights == [1 1 1 1 1 1]

        computeAndPlotTrajectorieswWithJacobianInJointSpace(puma, q, qd, qdd, trajTimes, eeName, angleThreshold, waypointTimes, jointWaypoints, includeOrientation,waypoints)%%  task space graphs preperation for includeOrientation = false
else
        PlotTrajectoriesNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold, waypoints, waypointTimes, jointWaypoints);
end

