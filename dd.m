%% Setup
clear, clc, close all

% Define waypoints and times (example values)
waypoints = [0.6 0.5 0 -0.8; -0.2 0 -0.25 -0.1; 0.6 0.3 1 0.65]; % Example waypoints
waypoints(:,4)=waypoints(:,1);
waypointTimes = [0 1 2 3]; % Time at each waypoint
waypointAccelTimes = [0.5 0.5 0.5]; % Acceleration time for each segment
max_joint_threshold=0.3;

% Define roll, pitch, and yaw angles for each waypoint
rollAngles = [0, pi/6, pi/4, pi/3];   % Roll angles (in radians)
pitchAngles = [0, -pi/12, pi/12, 0];  % Pitch angles (in radians)
yawAngles = [0, pi/8, -pi/8, pi/4];   % Yaw angles (in radians)

% Combine into the orientations matrix
orientations = [rollAngles; pitchAngles; yawAngles];

% Use a small sample time for this example
ts = 0.01;

% Define home joint angles (example values)
jointAnglesHome = [0 0 0 0 0 0]; % Home configuration for PUMA

% Load PUMA robot model
puma = loadrobot('puma560', 'DataFormat', 'row');

% Define end effector name
eeName = 'link7';

% Define threshold for small angles
angleThreshold = 0.0001;
% Define IK
ik = inverseKinematics('RigidBodyTree',puma);
ikWeights = [0 0 0 1 1 1];
ikInitGuess1 =jointAnglesHome;
ikInitGuess2=jointAnglesHome;
numJoints = numel(puma.homeConfiguration);

% Set up plot
plotMode = 2; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(puma, jointAnglesHome, 'Frames', 'off', 'PreservePlot', false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

% Initialize arrays for roll, pitch, yaw position, velocity, and acceleration

rollPos = [];allrollPos = [];
pitchPos = [];allpitchPos = [];
yawPos = [];allyawPos = [];
rollVel = [];allrollVel = [];
pitchVel = [];allpitchVel = [];
yawVel = [];allyawVel = [];
rollAcc = [];allrollAcc = [];
pitchAcc = [];allpitchAcc = [];
yawAcc = [];allyawAcc = [];
alleeAngularVelocities= [];
alleeAngularAccelerations= [];
% Initialize accumulators for trajectory data  
allTrajTimes = [];  alltrajTimesVel= [];  alltrajTimesAcc = []; 
allTaskPos = [];  
alltaskVel= [];
alltaskAcc= [];
config_FINAL= [];
 % config_all=zeros(numel(trajTimes),6);
%% Generate and follow trajectory
% Loop through segments one at a time
trajType = 'cubic';
numWaypoints = size(waypoints,2);
for w = 1:numWaypoints-1
    % Get the initial and final rotations and times for the segment
    R0 = eul2quat(orientations(:,w)');
    % Define a small tolerance for comparison


      if w==1
          R(:,1)= R0';
      end
        tolerance = 1e-3; % Adjust this value as needed

     % Check if eePos is close to any waypoint
       isMatch = any(all(abs(R(:,end)' - R0) < tolerance, 2));

      if ~isMatch
             R0=R(:,end)';
      end
    Rf = eul2quat(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);

    % Cartesian Motion only
    switch trajType
        case 'trap'
            [taskPos,taskVel,taskAcc] = trapveltraj(waypoints(:,w:w+1),numel(trajTimes), ...
                'AccelTime',waypointAccelTimes(w), ... 
                'EndTime',diff(waypointTimes(w:w+1)));

        case 'cubic'
            [taskPos,taskVel,taskAcc] = cubicpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',zeros(3,2));

        case 'quintic'
            [taskPos,taskVel,taskAcc] = quinticpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',zeros(3,2), ...
                'AccelerationBoundaryCondition',zeros(3,2));

        case 'bspline'
            ctrlpoints = waypoints(:,idx:idx+1); % Can adapt this as needed
            [taskPos,taskVel,taskAcc] = bsplinepolytraj(ctrlpoints,timeInterval,trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
        
    %% Find the quaternions from trajectory generation
    [R, omega, alpha] = rottraj(R0, Rf, timeInterval, trajTimes);    

      R(abs(R) < angleThreshold) = 0;
      omega(abs(omega) < angleThreshold) = 0;
      alpha(abs(alpha) < angleThreshold) = 0;

    % Convert quaternions to Euler angles (roll, pitch, yaw)
   % Convert quaternions to Euler angles (roll, pitch, yaw)
  eulAngles = quat2eul(R'); % Convert quaternion to Euler angles (roll, pitch, yaw)
  rollPos = eulAngles(:,1)';  % Roll position
  pitchPos =  eulAngles(:,2)'; % Pitch position
  yawPos =  eulAngles(:,3)';    % Yaw position

  % Angular velocity and acceleration (omega and alpha are in quaternion space)
  rollVel =  omega(1,:);  % Roll velocity
  pitchVel =  omega(2,:); % Pitch velocity
  yawVel =  omega(3,:);    % Yaw velocity

  rollAcc =  alpha(1,:);  % Roll acceleration
  pitchAcc =  alpha(2,:); % Pitch acceleration
  yawAcc =  alpha(3,:);    % Yaw acceleration
    TrajPoints = plot3(waypoints(1,w),waypoints(2,w),waypoints(3,w),'r.-');
  
    info.Status='success';
    is_significant_change=0;
    %% Trajectory following loop
    for idx = 1:numel(trajTimes) 
        % Solve IK
         disp([ num2str(idx)]);
         a= quat2tform(R(:,idx)');
         tgtPose = trvec2tform(taskPos(:,idx)') * a;


        

         if strcmp(info.Status, 'success')&& ~is_significant_change

                      
                     if idx >1 &&( w>1 || w==1)
 
                            ikInitGuess1=config_all(idx -1,:);
                             [config1,info1] = ik(eeName,tgtPose, [1 1 1 1 1 1],ikInitGuess1);
                              config_all(idx,:)=config1;
                              
                     
                     elseif idx ==1 && w>1
                    
                              config_all(idx,:)=config_all(end,:);
                              config1=config_all(end,:);
                              info1.Status='success';

                     elseif  idx ==1 && w==1
                              
                             [config1,info1] = ik(eeName,tgtPose, [1 1 1 1 1 1],ikInitGuess1);
                              config_all(idx,:)=config1;
                     end
            
            info.Status=info1.Status;

                if ~strcmp(info.Status, 'success')

                     if idx >1
                           ikInitGuess2= config_all(idx -1,:);
                     else
                          ikInitGuess2= config_all(end,:);
                     end
                    
                     [config2,info2] = ik(eeName,tgtPose, [0 0 0 1 1 1],ikInitGuess2);
                     ikInitGuess2 = config2;
                     config1= config2;
                     config_all(idx,:)=config1;
                 


                       
               end
            ikInitGuess1 = config1;
            if idx > 1
                     delta_config =config_all(idx,:) - config_all(idx-1,:);
                     change_magnitude = norm(delta_config);
                     max_joint_change = max(abs(delta_config));
                      disp([ num2str( max_joint_change)]);
                      is_significant_change =  (max_joint_change > max_joint_threshold);

                      if is_significant_change
                            idx_updated=idx-1;
                          [qT, qd, qdd, ikInitGuess] = generateTrajectory(idx, w, ts, trajType, ...  
                          waypointTimes, waypointAccelTimes, waypoints, taskPos, config_all, ...  
                          eeName, ik, numJoints, angleThreshold) ;
                           config_all(idx,:) = qT(:,(idx- idx_updated))';
    
                          % Find Cartesian points for visualization
                            eeTform = getTransform(puma, config_all(idx,:),eeName);
                           eePos_updated = tform2trvec(eeTform);
                      end

            end

             % Find Cartesian points for visualization
                            eeTform = getTransform(puma, config_all(idx,:),eeName);
                            eeANg=tform2quat(eeTform);
                           eulAngles_updated = quat2eul(eeANg);
                           rollPos(idx) =  eulAngles_updated(1,1)';  % Roll position
                           pitchPos(idx) =   eulAngles_updated(1,2)'; % Pitch position
                            yawPos(idx) =   eulAngles_updated(1,3)';    % Yaw po
                           eePos_updated = tform2trvec(eeTform);
          

        
         elseif ~strcmp(info.Status, 'success')&& ~is_significant_change
             ikInitGuess2= config_all(idx-1,:);
            [config2,info2] = ik(eeName,tgtPose, [0 0 0 1 1 1],ikInitGuess2);
             ikInitGuess2 = config2;
             config1= config2;
             config_all(idx,:)=config1;
             ikInitGuess1 = config1;
             if idx > 1
                     delta_config =config_all(idx,:) - config_all(idx-1,:);
                     change_magnitude = norm(delta_config);
                     max_joint_change = max(abs(delta_config));
                      disp([ num2str( max_joint_change)]);
                       is_significant_change =  (max_joint_change > max_joint_threshold);
                       if is_significant_change
                            idx_updated=idx-1;
                          [qT, qd, qdd, ikInitGuess] = generateTrajectory(idx, w, ts, trajType, ...  
                          waypointTimes, waypointAccelTimes, waypoints, taskPos, config_all, ...  
                          eeName, ik, numJoints, angleThreshold) ;
                           config_all(idx,:) = qT(:,(idx- idx_updated))';
    
                          % Find Cartesian points for visualization
                            eeTform = getTransform(puma, config_all(idx,:),eeName);
                           eePos_updated = tform2trvec(eeTform);
                      end
             end

              % Find Cartesian points for visualization
                            eeTform = getTransform(puma, config_all(idx,:),eeName);
                            eeANg=tform2quat(eeTform);
                           eulAngles_updated = quat2eul(eeANg);
                           rollPos(idx) =  eulAngles_updated(1,1)';  % Roll position
                           pitchPos(idx) =   eulAngles_updated(1,2)'; % Pitch position
                            yawPos(idx) =   eulAngles_updated(1,3)';    % Yaw po
                           eePos_updated = tform2trvec(eeTform);
         else
               config_all(idx,:) = qT(:,(idx- idx_updated))';
    
               % Find Cartesian points for visualization
               % Find Cartesian points for visualization
                            eeTform = getTransform(puma, config_all(idx,:),eeName);
                            eeANg=tform2quat(eeTform);
                           eulAngles_updated = quat2eul(eeANg);
                           rollPos(idx) =  eulAngles_updated(1,1)';  % Roll position
                           pitchPos(idx) =   eulAngles_updated(1,2)'; % Pitch position
                            yawPos(idx) =   eulAngles_updated(1,3)';    % Yaw po
                           eePos_updated = tform2trvec(eeTform);
          end
   

         % Define a small tolerance for comparison
        tolerance = 1e-3; % Adjust this value as needed

     % Check if eePos is close to any waypoint
       isMatch = any(all(abs(waypoints' - taskPos(:,idx)') < tolerance, 2));

    if isMatch
         % Find Cartesian points for visualization
         eeTform = getTransform(puma,config_all(idx,:),eeName);
         plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.1);
         
    end
    
       if  ~is_significant_change
       eePos = taskPos(:,idx)';
       set(TrajPoints,'xdata',[TrajPoints.XData eePos(1)], ...
                  'ydata',[TrajPoints.YData eePos(2)], ...
                  'zdata',[TrajPoints.ZData eePos(3)]);

       set(TrajPoints, 'Color', 'r'); % Set the trajectory color to black
       else
            taskPos(:,idx)=eePos_updated;
            set(TrajPoints,'xdata',[TrajPoints.XData eePos_updated(1)], ...
                  'ydata',[TrajPoints.YData eePos_updated(2)], ...
                  'zdata',[TrajPoints.ZData eePos_updated(3)]);

       set(TrajPoints, 'Color', 'r'); % Set the trajectory color to black

       end
       
        % Show the robot
        show(puma,config_all(idx,:),'Frames','off','PreservePlot',false);
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow    
    end

      eeOrientations = zeros(3, numel(trajTimes)); % End-effector orientations (roll, pitch, yaw)
    eeOrientations(1,:)=rollPos;
    eeOrientations(2,:)=pitchPos;
    eeOrientations(3,:)=yawPos;

    % Set small angles to zero
   taskPos(abs(taskPos) < angleThreshold) = 0;
   eeOrientations(abs( eeOrientations) < angleThreshold) = 0;
    % Compute task-space velocity (numerical differentiation)
   dt = mean(diff(trajTimes)); % Time step
   taskVel = diff(taskPos, 1, 2) / dt; % Velocity (x, y, z)
   taskVel = [zeros(3, 1), taskVel];  
   eeAngularVelocities = diff(eeOrientations, 1, 2) / dt; % Angular velocity (roll, pitch, yaw)
   eeAngularVelocities = [zeros(3, 1), eeAngularVelocities];  

   % Compute task-space acceleration (numerical differentiation)
    taskAcc = diff(taskVel, 1, 2) / dt; % Acceleration (x, y, z)
     taskAcc = [zeros(3, 1), taskAcc]; 
   eeAngularAccelerations = diff(eeAngularVelocities, 1, 2) / dt; % Angular acceleration (roll, pitch, yaw)
    eeAngularAccelerations = [zeros(3, 1), eeAngularAccelerations]; 

   
  rollVel=eeAngularVelocities(1,:);
  pitchVel=eeAngularVelocities(2,:);
  yawVel=eeAngularVelocities(3,:);
  rollAcc=eeAngularAccelerations(1,:);
  pitchAcc=eeAngularAccelerations(2,:);
  yawAcc=eeAngularAccelerations(3,:);

   % Adjust time vectors for velocity and acceleration
   trajTimesVel = trajTimes(1:end-1); % Time vector for velocity
   trajTimesAcc = trajTimes(1:end-2); % Time vector for acceleration

          % Accumulate trajectory data  
          allTrajTimes = [allTrajTimes, trajTimes];  
          alltrajTimesVel = [alltrajTimesVel, trajTimesVel];  
          alltrajTimesAcc = [alltrajTimesAcc, trajTimesAcc];  
          alleeAngularVelocities = [alleeAngularVelocities, eeAngularVelocities]; 
          alleeAngularAccelerations = [alleeAngularAccelerations, eeAngularAccelerations]; 
          config_FINAL = [config_FINAL; config_all];
          allTaskPos = [allTaskPos, taskPos];  
          alltaskVel= [alltaskVel, taskVel]; 
          alltaskAcc= [alltaskAcc,taskAcc]; 
          allrollPos= [allrollPos,rollPos];
          allpitchPos= [allpitchPos,pitchPos];
          allyawPos= [allyawPos,yawPos];
          allrollVel= [allrollVel,rollVel];
          allpitchVel= [allpitchVel,pitchVel];
         allyawVel= [allyawVel,yawVel];
         allrollAcc= [allrollAcc,rollAcc];
         allpitchAcc= [allpitchAcc,pitchAcc];
          allyawAcc= [allyawAcc,yawAcc];


    
end
numSteps= numel( allTrajTimes);
qd = zeros(numJoints, numSteps);   % Joint velocities
qdd = zeros(numJoints, numSteps);  % Joint accelerations
%% Compute Joint Velocities and Accelerations Using Inverse Jacobian and check the sigulirty
for jj = 1:numSteps
  % Get joint configuration at this time step
  config = config_FINAL(idx,:);

  % Compute Jacobian at the current configuration
  J = geometricJacobian(puma, config, eeName);
 
  if cond(J) > 1e6
  warning('Jacobian is near-singular at time step %d', jj);
  qd(:,jj) = zeros(numJoints, 1);
  qdd(:, jj) = zeros(numJoints, 1);
  continue;
  end
 % Extract task-space velocity and acceleration at this time step
  eeVel = [alleeAngularVelocities(:, jj); alltaskVel(:, jj)]; % Angular (orientation) + Linear (position) velocity
  eeAcc = [alleeAngularAccelerations(:, jj); alltaskAcc(:, jj)]; % Angular (orientation) + Linear (position) acceleration

  % Compute joint velocities using the inverse Jacobian
  qd(:, jj) = pinv(J) * eeVel; % Use pseudo-inverse in case J is not square

  % Compute joint accelerations using the inverse Jacobian
  if jj > 1
      % Approximate dJ/dt using finite differences
      J_prev = geometricJacobian(puma,config_FINAL(jj-1,: ), eeName);
      dJdt = (J - J_prev) / ( allTrajTimes(jj) -  allTrajTimes(jj-1));

      % Compute joint accelerations
      qdd(:, jj) = pinv(J) * eeAcc - pinv(J) * dJdt * qd(:, jj-1);
  else
      % For the first time step, assume zero acceleration
      qdd(:, jj) = zeros(numJoints, 1);
  end
end

%% Plot Task-Space Trajectory (Position, Velocity, Acceleration)


% Position
figure;
plot( allTrajTimes,  allTaskPos(1, :), 'r', 'LineWidth', 1.5); hold on;
plot( allTrajTimes,  allTaskPos(2, :), 'g', 'LineWidth', 1.5);
plot( allTrajTimes,  allTaskPos(3, :), 'b', 'LineWidth', 1.5);
title('Task-Space Position (x, y, z)');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');
grid on;

% Velocity
figure;
plot( allTrajTimes, alltaskVel(1, :), 'r', 'LineWidth', 1.5); hold on;
plot( allTrajTimes, alltaskVel(2, :), 'g', 'LineWidth', 1.5);
plot( allTrajTimes, alltaskVel(3, :), 'b', 'LineWidth', 1.5);
title('Task-Space Velocity (x, y, z)');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('x', 'y', 'z');
grid on;

% Acceleration
figure;
plot( allTrajTimes, alltaskAcc(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(allTrajTimes, alltaskAcc(2, :), 'g', 'LineWidth', 1.5);
plot( allTrajTimes, alltaskAcc(3, :), 'b', 'LineWidth', 1.5);
title('Task-Space Acceleration (x, y, z)');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('x', 'y', 'z');
grid on;

%% Plot Roll, Pitch, and Yaw Position, velocity and acceleration  in One Figure



% Orientation Position
figure;
plot(allTrajTimes,  allrollPos, 'r', 'LineWidth', 1.5); hold on;
plot(allTrajTimes, allpitchPos, 'g', 'LineWidth', 1.5);
plot(allTrajTimes, allyawPos, 'b', 'LineWidth', 1.5);
title('Orientation (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Orientation (rad)');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% Orientation Velocity
figure;
plot(allTrajTimes,  allrollVel, 'r', 'LineWidth', 1.5); hold on;
plot(allTrajTimes,   allpitchVel, 'g', 'LineWidth', 1.5);
plot(allTrajTimes, allyawVel, 'b', 'LineWidth', 1.5);
title('Orientation Velocity (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Roll', 'Pitch', 'Yaw');
grid on;

% Orientation Acceleration
figure;
plot(allTrajTimes,  allrollAcc, 'r', 'LineWidth', 1.5); hold on;
plot(allTrajTimes, allpitchAcc, 'g', 'LineWidth', 1.5);
plot(allTrajTimes, allyawAcc, 'b', 'LineWidth', 1.5);
title('Orientation Acceleration (Roll, Pitch, Yaw)');
xlabel('Time (s)');
ylabel('Angular Acceleration (rad/s^2)');
legend('Roll', 'Pitch', 'Yaw');
grid on;


%% Plot Joint-Space Trajectory


% Plot joint positions
figure;
plot(allTrajTimes,config_FINAL, 'LineWidth', 1.5);
title('Joint Positions');
xlabel('Time (s)');
ylabel('Position (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

% Plot joint velocities
figure;
plot(allTrajTimes, qd', 'LineWidth', 1.5);
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

% Plot joint accelerations
figure;
plot(allTrajTimes, qdd', 'LineWidth', 1.5);
title('Joint Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;







