function animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj, tolerance)
% ANIMATEROBOTTRAJECTORY Animates the robot following a trajectory and displays
% the traced end-effector path with coordinate frames at waypoints.
%
% Syntax:
%   animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj)
%   animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj, tolerance)
%
% Inputs:
%   puma      - RigidBodyTree robot model (e.g., puma560)
%   q         - numJoints x N matrix of joint angles over time
%   trajTimes - 1 x N vector of time steps
%   waypoints - 3 x M matrix of Cartesian waypoint positions [x; y; z]
%   eeName    - Name of the end-effector body (e.g., 'link7')
%   hTraj     - (Optional) Line handle of the trajectory path plot
%   tolerance - (Optional) Proximity tolerance to detect waypoints (default: 1e-3)

    if nargin < 7 || isempty(tolerance)
        tolerance = 1e-3;
    end

    if nargin < 6 || isempty(hTraj)
        hold on;
        hTraj = plot3(waypoints(1,1), waypoints(2,1), waypoints(3,1), 'b.-');
    end

    plotMode = 1;

    for idx = 1:numel(trajTimes)
        config = q(:, idx)';
        
        % Find Cartesian points for visualization
        eeTform = getTransform(puma, config, eeName);
        eePos = tform2trvec(eeTform);

        % Check if eePos is close to any waypoint
        isMatch = any(all(abs(waypoints' - eePos) < tolerance, 2));
        if isMatch
            plotMode = 2;
        end

        if plotMode == 1
            set(hTraj, 'xdata', [hTraj.XData eePos(1)], ...
                       'ydata', [hTraj.YData eePos(2)], ...
                       'zdata', [hTraj.ZData eePos(3)]);
        elseif plotMode == 2
            plotTransforms(tform2trvec(eeTform), tform2quat(eeTform), 'FrameSize', 0.1);
            plotMode = 1;
        end
        
        % Show the robot
        show(puma, config, 'Frames', 'off', 'PreservePlot', false);
        title(['Trajectory at t = ', num2str(trajTimes(idx))]);
        drawnow;
    end
end
