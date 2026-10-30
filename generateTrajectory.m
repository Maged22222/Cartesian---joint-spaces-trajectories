  
function [q, qd, qdd, ikInitGuess] = generateTrajectory(idx, w, ts, trajType, ...  
    waypointTimes, waypointAccelTimes, waypoints, taskPos, config_all, ...  
    eeName, ik, numJoints, angleThreshold)  
    
    % Number of waypoints to update  
    update_numWaypoints = 2;  
    update_Waypoints_time(1) =  waypointTimes(w); 
    update_Waypoints_time(2) =  waypointTimes(w+1);
    waypointAccelTimes_updated=  waypointAccelTimes(w) ;

    % Initialize the initial guess for IK  
    ikInitGuess = config_all(idx - 1, :);  

    % Update the time interval for the current segment  
    timeInterval = waypointTimes(w:w+1);  
    timeInterval(1) = timeInterval(1) + ts * (idx - 1);  
    trajTimes_update = timeInterval(1):ts:timeInterval(2);  

    % Initialize joint waypoints  
    jointWaypoints = zeros(numJoints, update_numWaypoints);  

    % Loop through the waypoints to compute joint configurations  
    for ii = 1:update_numWaypoints  
        if ii == 1  
            % Use the current task position for the first waypoint  
            tgtPose1 = trvec2tform(taskPos(:, idx)');  
        else  
            % Use the next waypoint for the second waypoint  
            tgtPose1 = trvec2tform(waypoints(:, w + 1)');  
        end  

        % Solve IK for the target pose  
        [config, info] = ik(eeName, tgtPose1, [0 0 0 1 1 1], ikInitGuess);  
        ikInitGuess = config; % Update the initial guess for the next iteration  
        jointWaypoints(:, ii) = config'; % Store the joint configuration  
    end  

    % Generate the trajectory based on the specified type  
    switch trajType  
        case 'trap'  
            % Trapezoidal velocity trajectory  
            [q, qd, qdd] = trapveltraj(jointWaypoints, numel(trajTimes_update), ...  
                'AccelTime', repmat( waypointAccelTimes_updated, [numJoints 1]), ...  
                'EndTime', repmat(diff(update_Waypoints_time), [numJoints 1]));  

        case 'cubic'  
            % Cubic polynomial trajectory  
            [q, qd, qdd] = cubicpolytraj(jointWaypoints,update_Waypoints_time, trajTimes_update, ...  
                'VelocityBoundaryCondition', zeros(numJoints, update_numWaypoints));  

        case 'quintic'  
            % Quintic polynomial trajectory  
            [q, qd, qdd] = quinticpolytraj(jointWaypoints, update_Waypoints_time, trajTimes_update, ...  
                'VelocityBoundaryCondition', zeros(numJoints, update_numWaypoints), ...  
                'AccelerationBoundaryCondition', zeros(numJoints, update_numWaypoints));  

        case 'bspline'  
            % B-spline trajectory  
            ctrlpoints = jointWaypoints; % Control points for B-spline  
            [q, qd, qdd] = bsplinepolytraj(ctrlpoints, update_Waypoints_time([1 end]), trajTimes_update);  

        otherwise  
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');  
    end  

    % Set small angles to zero  
    q(abs(q) < angleThreshold) = 0;  
end  