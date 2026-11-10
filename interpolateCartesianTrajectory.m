function [taskPos, taskVel, taskAcc, taskOrient, taskOrientVel, taskOrientAcc] = ...
    interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition, ...
    orientations, rentaiontrajectorytype, activateorentation)
% INTERPOLATECARTESIANTRAJECTORY Generates Cartesian-space position and optional 
% orientation trajectories (velocity, acceleration included).
%
% Syntax:
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes)
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes)
%   [taskPos, taskVel, taskAcc] = interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition)
%   [taskPos, taskVel, taskAcc, taskOrient, taskOrientVel, taskOrientAcc] = ...
%       interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, waypointAccelTimes, ...
%                                      velocityBoundaryCondition, accelerationBoundaryCondition, ...
%                                      orientations, rentaiontrajectorytype, activateorentation)
%
% Inputs:
%   trajType                      - Trajectory type: 'trap', 'cubic', 'quintic', or 'bspline'
%   waypoints                     - M x numWaypoints matrix of Cartesian waypoints (e.g., 3 x numWaypoints)
%   waypointTimes                 - 1 x numWaypoints vector of waypoint arrival times
%   trajTimes                     - 1 x N vector of query time stamps
%   waypointAccelTimes            - (Optional) Acceleration times for trapezoidal velocity profile
%   velocityBoundaryCondition     - (Optional) M x numWaypoints matrix of velocity boundary conditions (default: zeros)
%   accelerationBoundaryCondition - (Optional) M x numWaypoints matrix of acceleration boundary conditions (default: zeros)
%   orientations                  - (Optional) numWaypoints x 3 (or 3 x numWaypoints) Euler angles [roll, pitch, yaw]
%   rentaiontrajectorytype        - (Optional) 'trajType' (uses trajType on Euler angles) or 'rottraj' (SO(3) quaternion trajectory). Default: 'trajType'.
%   activateorentation            - (Optional) 1 to generate orientation, 0 for position only. Default: true if orientations provided.
%
% Outputs:
%   taskPos       - 3 x N matrix of Cartesian positions
%   taskVel       - 3 x N matrix of Cartesian linear velocities
%   taskAcc       - 3 x N matrix of Cartesian linear accelerations
%   taskOrient    - 3 x N matrix of end-effector Euler orientations [roll; pitch; yaw] (empty if activateorentation = 0)
%   taskOrientVel - 3 x N matrix of angular velocities (rad/s) (empty if activateorentation = 0)
%   taskOrientAcc - 3 x N matrix of angular accelerations (rad/s^2) (empty if activateorentation = 0)

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

    if nargin < 9 || isempty(rentaiontrajectorytype)
        rentaiontrajectorytype = 'trajType';
    end

    %% 1. Cartesian Position Trajectory Generation
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

    %% 2. Cartesian Orientation Trajectory Generation (if orientations provided and activated)
    if nargin < 10 || isempty(activateorentation)
        activateorentation = (nargin >= 8 && ~isempty(orientations));
    else
        activateorentation = logical(activateorentation);
    end

    if activateorentation && nargin >= 8 && ~isempty(orientations)
        % Format orientations to 3 x numWaypoints
        if size(orientations, 1) == numWaypoints && size(orientations, 2) == 3
            orientWaypoints = orientations';
        else
            orientWaypoints = orientations;
        end

        if strcmpi(rentaiontrajectorytype, 'rottraj')
            % Option A: rottraj across waypoint segments (SO(3) geodesic / quaternions)
            taskOrient = zeros(3, numTimeSteps);
            taskOrientVel = zeros(3, numTimeSteps);
            taskOrientAcc = zeros(3, numTimeSteps);

            for seg = 1:numWaypoints - 1
                t0 = waypointTimes(seg);
                tf = waypointTimes(seg + 1);
                if seg == numWaypoints - 1
                    segIdx = find(trajTimes >= t0 & trajTimes <= tf);
                else
                    segIdx = find(trajTimes >= t0 & trajTimes < tf);
                end
                tSamples = trajTimes(segIdx);

                R0 = eul2rotm(orientWaypoints(:, seg)');
                Rf = eul2rotm(orientWaypoints(:, seg + 1)');

                [R_seg, omega_seg, alpha_seg] = rottraj(R0, Rf, [t0, tf], tSamples);

                for s = 1:numel(segIdx)
                    taskOrient(:, segIdx(s)) = rotm2eul(R_seg(:, :, s))';
                end
                taskOrientVel(:, segIdx) = omega_seg;
                taskOrientAcc(:, segIdx) = alpha_seg;
            end
            taskOrient = unwrap(taskOrient, [], 2);

        else
            % Option B: Inherit trajType (polynomial / trapveltraj on Euler angles)
            orientDim = size(orientWaypoints, 1);
            switch lower(trajType)
                case 'trap'
                    [taskOrient, taskOrientVel, taskOrientAcc] = trapveltraj(orientWaypoints, numTimeSteps, ...
                        'AccelTime', repmat(waypointAccelTimes, [orientDim, 1]), ...
                        'EndTime', repmat(diff(waypointTimes), [orientDim, 1]));

                case 'cubic'
                    [taskOrient, taskOrientVel, taskOrientAcc] = cubicpolytraj(orientWaypoints, waypointTimes, trajTimes, ...
                        'VelocityBoundaryCondition', zeros(orientDim, numWaypoints));

                case 'quintic'
                    [taskOrient, taskOrientVel, taskOrientAcc] = quinticpolytraj(orientWaypoints, waypointTimes, trajTimes, ...
                        'VelocityBoundaryCondition', zeros(orientDim, numWaypoints), ...
                        'AccelerationBoundaryCondition', zeros(orientDim, numWaypoints));

                case 'bspline'
                    ctrlpointsOrient = orientWaypoints;
                    [taskOrient, taskOrientVel, taskOrientAcc] = bsplinepolytraj(ctrlpointsOrient, waypointTimes([1 end]), trajTimes);

                otherwise
                    error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
            end
            taskOrient = unwrap(taskOrient, [], 2);
        end
    else
        % Orientation deactivated or not provided
        taskOrient = [];
        taskOrientVel = [];
        taskOrientAcc = [];
    end
end
