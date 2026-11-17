function [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName, lambda, condThreshold, velThreshold)

 
    numJoints = size(q, 1);
    numSteps = size(q, 2);
    qd = zeros(numJoints, numSteps);
    qdd = zeros(numJoints, numSteps);

    epsilon = 0.005; 

    for idx = 1:numSteps
 
        config = q(:, idx)';
        J = geometricJacobian(puma, config, eeName);
        if lambda > 0
            [U, S, V] = svd(J, 'econ');
            s = diag(S);
            sigma_min = min(s);
            if sigma_min < epsilon
                lam2 = lambda^2 * (1 - (sigma_min / epsilon)^2);
            else
                lam2 = 0; 
            end

            s_dls = s ./ (s.^2 + lam2);
            J_inv = V * diag(s_dls) * U';
        else
            if cond(J) > condThreshold
                warning('Jacobian is near-singular at time step %d (condition number = %.2e)', idx, cond(J));
                qd(:, idx) = zeros(numJoints, 1);
                qdd(:, idx) = zeros(numJoints, 1);
                continue;
            end
            J_inv = pinv(J);
        end

        qd(:, idx) = J_inv * eeVel(:, idx);
        zeroVelMask = abs(qd(:, idx)) < velThreshold;
        qd(zeroVelMask, idx) = 0;

        h = 1e-6;
        currentJointVel = qd(:, idx)';
        Jplus  = geometricJacobian(puma, config + h * currentJointVel, eeName);
        Jminus = geometricJacobian(puma, config - h * currentJointVel, eeName);
        Jdot = (Jplus - Jminus) / (2 * h);
        qdd(:, idx) = J_inv * (eeAcc(:, idx) - Jdot * qd(:, idx));
        qdd(zeroVelMask, idx) = 0;
    end

    zeroVelAll = abs(qd) < velThreshold;
    qd(zeroVelAll) = 0;
    qdd(zeroVelAll) = 0;
end
