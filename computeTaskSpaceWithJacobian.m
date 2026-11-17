function [eePos, eeOrient, eeVel, eeAcc] = computeTaskSpaceWithJacobian(puma, q, qd, qdd, eeName, angleThreshold)


   

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
        eeTform = getTransform(puma, config, eeName);
        eePos(:, idx) = tform2trvec(eeTform)';
        eeOrient(:, idx) = tform2eul(eeTform)';

        if angleThreshold > 0
            eePos(abs(eePos) < angleThreshold) = 0;
            eeOrient(abs(eeOrient) < angleThreshold) = 0;
        end
        J = geometricJacobian(puma, config, eeName);
        eeVel(1:3, idx) = J(1:3, :) * jointVel';
        eeVel(4:6, idx) = J(4:6, :) * jointVel';

        h = 1e-6;
        Jplus  = geometricJacobian(puma, config + h * jointVel, eeName);
        Jminus = geometricJacobian(puma, config - h * jointVel, eeName);
        Jdot = (Jplus - Jminus) / (2 * h);

        eeAcc(:, idx) = J * jointAcc' + Jdot * jointVel';
    end

    eeOrient = unwrap(eeOrient, [], 2);
end
