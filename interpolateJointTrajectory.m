function [q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes, angleThreshold)
% INTERPOLATEJOINTTRAJECTORY Generates joint-space position, velocity, and acceleration trajectories.
%
% Syntax:
%   [q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes)
%   [q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes)
%   [q, qd, qdd] = interpolateJointTrajectory(trajType, jointWaypoints, waypointTimes, trajTimes, waypointAccelTimes, angleThreshold)
%
% Inputs:
%   trajType           - Trajectory type: 'trap', 'cubic', 'quintic', or 'bspline'
%   jointWaypoints     - numJoints x numWaypoints matrix of joint configurations at waypoints
%   waypointTimes      - 1 x numWaypoints vector of waypoint arrival times
%   trajTimes          - 1 x N vector of query time stamps
%   waypointAccelTimes - (Optional) Acceleration times for trapezoidal velocity profile
%   angleThreshold     - (Optional) Threshold to zero out near-zero joint angles
%
% Outputs:
%   q   - numJoints x N matrix of joint positions
%   qd  - numJoints x N matrix of joint velocities
%   qdd - numJoints x N matrix of joint accelerations

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

    % Set small angles to zero if threshold provided
    if nargin >= 6 && ~isempty(angleThreshold)
        q(abs(q) < angleThreshold) = 0;
    end
end
