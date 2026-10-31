function [eePositions, eeOrientations, eeVelocities, eeAngularVelocities, ...
          eeAccelerations, eeAngularAccelerations, trajTimesVel, trajTimesAcc] = ...
          computeTaskSpaceNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold)
% COMPUTETASKSPACENUMERICALDIFFERENTIATION Computes task-space positions, orientations,
% and calculates velocities and accelerations via numerical differentiation of Forward Kinematics.
%
% Syntax:
%   [eePositions, eeOrientations, eeVelocities, eeAngularVelocities, eeAccelerations, eeAngularAccelerations, trajTimesVel, trajTimesAcc] = ...
%       computeTaskSpaceNumericalDifferentiation(puma, q, trajTimes, eeName)
%   [eePositions, eeOrientations, eeVelocities, eeAngularVelocities, eeAccelerations, eeAngularAccelerations, trajTimesVel, trajTimesAcc] = ...
%       computeTaskSpaceNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold)
%
% Inputs:
%   puma                    - RigidBodyTree robot model (e.g. puma560)
%   q                       - numJoints x numTimeSteps matrix of joint angles
%   trajTimes               - 1 x numTimeSteps vector of timestamps
%   eeName                  - End-effector body name (e.g. 'link7')
%   angleThreshold          - (Optional) Threshold to zero out small values (default: 0)
%
% Outputs:
%   eePositions             - 3 x numTimeSteps end-effector positions (x, y, z)
%   eeOrientations          - 3 x numTimeSteps end-effector Euler orientations (roll, pitch, yaw)
%   eeVelocities            - 3 x (numTimeSteps-1) linear velocities (dx/dt, dy/dt, dz/dt)
%   eeAngularVelocities     - 3 x (numTimeSteps-1) angular velocities
%   eeAccelerations         - 3 x (numTimeSteps-2) linear accelerations (d2x/dt2, ...)
%   eeAngularAccelerations  - 3 x (numTimeSteps-2) angular accelerations
%   trajTimesVel            - 1 x (numTimeSteps-1) time vector for velocities
%   trajTimesAcc            - 1 x (numTimeSteps-2) time vector for accelerations

    if nargin < 5 || isempty(angleThreshold)
        angleThreshold = 0;
    end

    numTimeSteps = numel(trajTimes);
    eePositions = zeros(3, numTimeSteps);
    eeOrientations = zeros(3, numTimeSteps);

    % Compute forward kinematics for each time step
    for tIdx = 1:numTimeSteps
        jointConfig = q(:, tIdx)';
        eePose = getTransform(puma, jointConfig, eeName);
        eePositions(:, tIdx) = tform2trvec(eePose);
        eeOrientations(:, tIdx) = tform2eul(eePose);
    end

    % Set small angles to zero
    if angleThreshold > 0
        eePositions(abs(eePositions) < angleThreshold) = 0;
        eeOrientations(abs(eeOrientations) < angleThreshold) = 0;
    end

    % Compute task-space velocity (numerical differentiation)
    dt = mean(diff(trajTimes));
    eeVelocities = diff(eePositions, 1, 2) / dt;
    eeAngularVelocities = diff(eeOrientations, 1, 2) / dt;

    % Compute task-space acceleration (numerical differentiation)
    eeAccelerations = diff(eeVelocities, 1, 2) / dt;
    eeAngularAccelerations = diff(eeAngularVelocities, 1, 2) / dt;

    % Adjust time vectors for velocity and acceleration
    trajTimesVel = trajTimes(1:end-1);
    trajTimesAcc = trajTimes(1:end-2);
end
