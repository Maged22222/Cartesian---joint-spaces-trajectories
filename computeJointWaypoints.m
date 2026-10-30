function [jointWaypoints, ikInitGuess] = computeJointWaypoints(ik, eeName, waypoints, orientations, includeOrientation, ikWeights, ikInitGuess)
% COMPUTEJOINTWAYPOINTS Solves Inverse Kinematics for a sequence of waypoints.
%
% Syntax:
%   jointWaypoints = computeJointWaypoints(ik, eeName, waypoints, orientations, includeOrientation, ikWeights, ikInitGuess)
%   [jointWaypoints, ikInitGuess] = computeJointWaypoints(...)
%
% Inputs:
%   ik                 - Inverse kinematics solver object (inverseKinematics)
%   eeName             - Name of the end-effector body (e.g., 'link7')
%   waypoints          - 3 x N matrix of Cartesian waypoint positions [x; y; z]
%   orientations       - N x 3 matrix of Euler angles [roll, pitch, yaw]
%   includeOrientation - Logical flag: true to match orientation, false for position only
%   ikWeights          - 1 x 6 vector of IK weights [orient_weights, pos_weights]
%   ikInitGuess        - 1 x numJoints initial joint configuration guess
%
% Outputs:
%   jointWaypoints     - numJoints x N matrix of joint angles for each waypoint
%   ikInitGuess        - Final joint configuration reached (can be used as seed for next call)

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
