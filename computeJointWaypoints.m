function [jointWaypoints, ikInitGuess] = computeJointWaypoints(ik, eeName, waypoints, orientations, includeOrientation, ikWeights, ikInitGuess)


    numWaypoints = size(waypoints, 2);
    numJoints = numel(ikInitGuess);
    jointWaypoints = zeros(numJoints, numWaypoints);

    for idx = 1:numWaypoints
        if includeOrientation
            tgtPose = trvec2tform(waypoints(:, idx)') * eul2tform(orientations(idx, :));
        else
            tgtPose = trvec2tform(waypoints(:, idx)');
        end
        
        [config, ~] = ik(eeName, tgtPose, ikWeights, ikInitGuess);
        ikInitGuess = config;
        jointWaypoints(:, idx) = config';
    end
end
