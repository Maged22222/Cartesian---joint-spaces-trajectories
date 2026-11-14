function [eePos, eeOrient, eeVel, eeAcc] = computeTaskSpaceWithJacobian(puma, q, qd, qdd, eeName, angleThreshold)


    if nargin < 6 || isempty(angleThreshold)
        angleThreshold = 0;
    end

    numSteps = size(q, 2);
    eePos = zeros(3, numSteps);
    eeOrient = zeros(3, numSteps);
    eeVel = zeros(6, numSteps);
    eeAcc = zeros(6, numSteps);

    for idx = 1:numSteps
        config = q(:, idx)';
        jointVel = qd(:, idx)';
        jointAcc = qdd(:, idx)';

        if angleThreshold > 0
            jointVel(abs(jointVel) < angleThreshold) = 0;
            jointAcc(abs(jointAcc) < angleThreshold) = 0;
        end

        % Compute forward kinematics to get end-effector position and orientation
        eeTform = getTransform(puma, config, eeName);
        eePos(:, idx) = tform2trvec(eeTform)';
        eeOrient(:, idx) = tform2eul(eeTform)';

        if angleThreshold > 0
            eePos(abs(eePos) < angleThreshold) = 0;
            eeOrient(abs(eeOrient) < angleThreshold) = 0;
        end

        % Compute Jacobian at the current configuration
        J = geometricJacobian(puma, config, eeName);

        % Compute task-space velocity: eeVel (Angular + Linear)
        eeVel(1:3, idx) = J(1:3, :) * jointVel';
        eeVel(4:6, idx) = J(4:6, :) * jointVel';

        % Time derivative of the Jacobian along the motion: dJ/dt = sum_i (dJ/dq_i) * qd_i
        % Computed as a central directional derivative of J in the direction of qd
        % (exact up to O(h^2), independent of the trajectory sample time).
        h = 1e-6;
        Jplus  = geometricJacobian(puma, config + h * jointVel, eeName);
        Jminus = geometricJacobian(puma, config - h * jointVel, eeName);
        Jdot = (Jplus - Jminus) / (2 * h);

        % Compute task-space acceleration: eeAcc = J*qdd + Jdot*qd (Angular + Linear)
        % NOTE: the Jdot*qd term (centripetal/Coriolis part) is required; omitting it
        % makes the Jacobian acceleration disagree with numerical differentiation.
        eeAcc(:, idx) = J * jointAcc' + Jdot * jointVel';
    end

    % Remove +/-pi wrapping of Euler angles (same convention as the numerical method)
    eeOrient = unwrap(eeOrient, [], 2);
end
