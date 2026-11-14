function [eePositions, eeOrientations, eeVelocities, eeAngularVelocities, ...
          eeAccelerations, eeAngularAccelerations, trajTimesVel, trajTimesAcc] = ...
          computeTaskSpaceNumericalDifferentiation(puma, q, trajTimes, eeName, angleThreshold)


    if nargin < 5 || isempty(angleThreshold)
        angleThreshold = 0;
    end

    numTimeSteps = numel(trajTimes);
    eePositions = zeros(3, numTimeSteps);
    eeOrientations = zeros(3, numTimeSteps);
    eeRotm = zeros(3, 3, numTimeSteps);

    % Compute forward kinematics for each time step
    for tIdx = 1:numTimeSteps
        jointConfig = q(:, tIdx)';
        eePose = getTransform(puma, jointConfig, eeName);
        eePositions(:, tIdx) = tform2trvec(eePose);
        eeOrientations(:, tIdx) = tform2eul(eePose);
        eeRotm(:, :, tIdx) = tform2rotm(eePose);
    end

    % Remove +/-pi wrapping of Euler angles (avoids huge fake spikes)
    eeOrientations = unwrap(eeOrientations, [], 2);

    % Set small values to zero
    if angleThreshold > 0
        eePositions(abs(eePositions) < angleThreshold) = 0;
        eeOrientations(abs(eeOrientations) < angleThreshold) = 0;
    end

    dt = mean(diff(trajTimes));

    % Linear velocity / acceleration (central differences, time-aligned)
    eeVelocities = centralDiff(eePositions, dt);
    eeAccelerations = centralDiff(eeVelocities, dt);

    % Angular velocity from rotation matrices: [omega]x = dR/dt * R'
    % Using the relative rotation between neighbouring samples:
    %   R(k+1) * R(k-1)' = expm([omega]x * 2*dt)  ->  omega = axis*angle / (2*dt)
    eeAngularVelocities = zeros(3, numTimeSteps);
    for tIdx = 1:numTimeSteps
        iPrev = max(tIdx - 1, 1);
        iNext = min(tIdx + 1, numTimeSteps);
        dRel = eeRotm(:, :, iNext) * eeRotm(:, :, iPrev)';
        axang = rotm2axang(dRel);
        eeAngularVelocities(:, tIdx) = axang(1:3)' * axang(4) / ((iNext - iPrev) * dt);
    end
    eeAngularAccelerations = centralDiff(eeAngularVelocities, dt);

    % Time vectors (same length as trajTimes because central differences are used)
    trajTimesVel = trajTimes;
    trajTimesAcc = trajTimes;
end

function dX = centralDiff(X, dt)
% Central difference along columns (one-sided at the ends), same size as X.
    dX = zeros(size(X));
    if size(X, 2) < 2
        return;
    end
    dX(:, 2:end-1) = (X(:, 3:end) - X(:, 1:end-2)) / (2 * dt);
    dX(:, 1) = (X(:, 2) - X(:, 1)) / dt;
    dX(:, end) = (X(:, end) - X(:, end-1)) / dt;
end
