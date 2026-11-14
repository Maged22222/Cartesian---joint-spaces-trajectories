function animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj, tolerance)


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
