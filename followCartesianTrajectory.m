function [config_all, ikInitGuess] = followCartesianTrajectory(puma, eeName, ik, ikWeights, ikInitGuess, taskPos, trajTimes, waypoints, TrajPoints, angleThreshold, tolerance, taskOrient, activateorentation)

    numSteps = numel(trajTimes);
    numJoints = numel(puma.homeConfiguration);
    hasTrajPoints = (nargin >= 9 && ~isempty(TrajPoints) && isgraphics(TrajPoints));

    config_all = zeros(numSteps, numJoints);
    config = ikInitGuess;

    for idx = 1:numSteps
       
        tgtPose = trvec2tform(taskPos(:, idx)');
        [config, info] = ik(eeName, tgtPose, ikWeights, ikInitGuess);
        ikInitGuess = config;
        config(abs(config) < angleThreshold) = 0;
        config_all(idx, :) = config;

        if ~strcmp(info.Status, 'success')
            warning('IK solver failed at time step %d', idx);
        end

        eePos = taskPos(:, idx)';
        if hasTrajPoints
            set(TrajPoints, 'xdata', [TrajPoints.XData, eePos(1)], ...
                            'ydata', [TrajPoints.YData, eePos(2)], ...
                            'zdata', [TrajPoints.ZData, eePos(3)]);
            set(TrajPoints, 'Color', 'r'); 
        end
        isMatch = any(all(abs(waypoints' - taskPos(:, idx)') < angleThreshold, 2));

        if isMatch
            eeTform = getTransform(puma, config, eeName);
            plotTransforms(tform2trvec(eeTform), tform2quat(eeTform), 'FrameSize', 0.1);
        end
        show(puma, config_all(idx, :), 'Frames', 'off', 'PreservePlot', false);
        title(['Trajectory at t = ', num2str(trajTimes(idx))]);
        drawnow;
    end
end
