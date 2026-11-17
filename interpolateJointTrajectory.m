function [q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes, angleThreshold)


    [numJoints, numWaypoints] = size(jointWaypoints);
    numTimeSteps = numel(trajTimes);

    switch lower(trajType)
        case 'trap'
            if nargin < 5 || isempty(waypointAccelTimes)
                waypointAccelTimes = repmat(0.5, [1, numWaypoints - 1]);
            end
            [q, qd, qdd] = trapveltraj(jointWaypoints, numTimeSteps, ...
                'AccelTime', repmat(waypointAccelTimes, [numJoints, 1]), ...
                'EndTime', repmat(diff(waypointTimes), [numJoints, 1]));

        case 'cubic'
            [q, qd, qdd] = cubicpolytraj(jointWaypoints, waypointTimes, trajTimes, ...
                'VelocityBoundaryCondition', zeros(numJoints, numWaypoints));

        case 'quintic'
            [q, qd, qdd] = quinticpolytraj(jointWaypoints, waypointTimes, trajTimes, ...
                'VelocityBoundaryCondition', zeros(numJoints, numWaypoints), ...
                'AccelerationBoundaryCondition', zeros(numJoints, numWaypoints));

        case 'bspline'
            ctrlpoints = jointWaypoints;
            [q, qd, qdd] = bsplinepolytraj(ctrlpoints, waypointTimes([1 end]), trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end

    
    if nargin >= 6 && ~isempty(angleThreshold)
        q(abs(q) < angleThreshold) = 0;
    end
end
