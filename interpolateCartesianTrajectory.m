function [taskPos, taskVel, taskAcc, taskOrient, taskOrientVel, taskOrientAcc] = ...
    interpolateCartesianTrajectory(trajType, waypoints, waypointTimes, trajTimes, ...
    waypointAccelTimes, velocityBoundaryCondition, accelerationBoundaryCondition, ...
    orientations, rentaiontrajectorytype, activateorentation)

    [dim, numWaypoints] = size(waypoints);
    numTimeSteps = numel(trajTimes);

    %% Cartesian Position Trajectory Generation
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

    %% Cartesian Orientation Trajectory Generation 
    
    if activateorentation && nargin >= 8 && ~isempty(orientations)

           orientWaypoints = orientations';

        if strcmpi(rentaiontrajectorytype, 'rottraj')
           
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
   
        taskOrient = [];
        taskOrientVel = [];
        taskOrientAcc = [];
    end
end
