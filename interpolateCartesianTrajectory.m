function [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition)
% INTERPOLATECARTESIANTRAJECTORY Generates Cartesian-space position, velocity, and acceleration trajectories.
%
% Syntax:
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes)
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes)
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes, velocityBoundaryCondition)
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition)
%
% Inputs:
%   trajType                      - Trajectory type: 'trap', 'cubic', 'quintic', or 'bspline'
%   waypoints                     - M x numWaypoints matrix of Cartesian waypoints (e.g., 3 x numWaypoints)
%   waypointTimes                 - 1 x numWaypoints vector of waypoint arrival times
%   trajTimes                     - 1 x N vector of query time stamps
%   waypointAccelTimes            - (Optional) Acceleration times for trapezoidal velocity profile
%   velocityBoundaryCondition     - (Optional) M x numWaypoints matrix of velocity boundary conditions (default: zeros)
%   accelerationBoundaryCondition - (Optional) M x numWaypoints matrix of acceleration boundary conditions (default: zeros)
%
% Outputs:
%   taskPos - M x N matrix of Cartesian positions
%   taskVel - M x N matrix of Cartesian linear velocities
%   taskAcc - M x N matrix of Cartesian linear accelerations

    [dim, numWaypoints] = size(waypoints);
    numTimeSteps = numel(trajTimes);

    if nargin < 5 || isempty(waypointAccelTimes)
        waypointAccelTimes = repmat(0.5, [1, numWaypoints - 1]);
    end

    if nargin < 6 || isempty(velocityBoundaryCondition)
        velocityBoundaryCondition = zeros(dim, numWaypoints);
    end

    if nargin < 7 || isempty(accelerationBoundaryCondition)
        accelerationBoundaryCondition = zeros(dim, numWaypoints);
    end

    switch lower(trajType)
        case 'trap'
            [taskPos, taskVel, taskAcc] = trapveltraj(waypoints, numTimeSteps, ...
                'AccelTime', repmat(waypointAccelTimes, [dim, 1]), ...
                'EndTime', repmat(diff(waypointTimes), [dim, 1]));

        case 'cubic'
            [taskPos, taskVel, taskAcc] = cubicpolytraj(waypoints, waypointTimes, trajTimes, ...
                'VelocityBoundaryCondition', velocityBoundaryCondition);

        case 'quintic'
            [taskPos, taskVel, taskAcc] = quinticpolytraj(waypoints, waypointTimes, trajTimes, ...
                'VelocityBoundaryCondition', velocityBoundaryCondition, ...
                'AccelerationBoundaryCondition', accelerationBoundaryCondition);

        case 'bspline'
            ctrlpoints = waypoints;
            [taskPos, taskVel, taskAcc] = bsplinepolytraj(ctrlpoints, waypointTimes([1 end]), trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
end
