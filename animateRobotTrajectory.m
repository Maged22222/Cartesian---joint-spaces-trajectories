function animateRobotTrajectory(puma, q, trajTimes, waypoints, eeName, hTraj, tolerance)
    plotMode = 1;

    for idx = 1:numel(trajTimes)
        config = q(:, idx)';
        eeTform = getTransform(puma, config, eeName);
        eePos = tform2trvec(eeTform);
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
        show(puma, config, 'Frames', 'off', 'PreservePlot', false);
        title(['Trajectory at t = ', num2str(trajTimes(idx))]);
        drawnow;
    end
end
