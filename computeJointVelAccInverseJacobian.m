function [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName, lambda, condThreshold, velThreshold)


    if nargin < 7 || isempty(lambda)
        lambda = 0.05; % Default max DLS damping factor
    end

    if nargin < 8 || isempty(condThreshold) || condThreshold < 1
        condThreshold = 1e6;
    end

    if nargin < 9 || isempty(velThreshold)
        velThreshold = 0.1; % Cutoff threshold: values less than 0.1 are set to zero
    end

    % If only 3-row linear vectors are provided, prepend 3 zeros for angular components
    if size(eeVel, 1) == 3
        eeVel = [zeros(3, size(eeVel, 2)); eeVel];
    end
    if size(eeAcc, 1) == 3
        eeAcc = [zeros(3, size(eeAcc, 2)); eeAcc];
    end

    numJoints = size(q, 1);
    numSteps = size(q, 2);

    qd = zeros(numJoints, numSteps);
    qdd = zeros(numJoints, numSteps);

    epsilon = 0.005; % Singularity threshold for adaptive damping

    for idx = 1:numSteps
        % Get joint configuration at this time step
        config = q(:, idx)';

        % Compute Jacobian at the current configuration
        J = geometricJacobian(puma, config, eeName);

        % Compute Jacobian inverse using Singularity-Adaptive DLS
        if lambda > 0
            [U, S, V] = svd(J, 'econ');
            s = diag(S);
            sigma_min = min(s);

            % Adaptive damping: only active when approaching singularity (sigma_min < epsilon)
            if sigma_min < epsilon
                lam2 = lambda^2 * (1 - (sigma_min / epsilon)^2);
            else
                lam2 = 0; % Exact un-damped inversion
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

        % Compute joint velocities using the DLS inverse Jacobian
        qd(:, idx) = J_inv * eeVel(:, idx);

        % If abs(qd(:, idx)) < velThreshold is true, zero out qd and qdd for the same index
        zeroVelMask = abs(qd(:, idx)) < velThreshold;
        qd(zeroVelMask, idx) = 0;

        % Time derivative of the Jacobian along the motion: dJ/dt = sum_i (dJ/dq_i) * qd_i
        % Computed as a central directional derivative in the direction of qd
        % (exact up to O(h^2), valid for all time steps including idx = 1, independent of dt).
        h = 1e-6;
        currentJointVel = qd(:, idx)';
        Jplus  = geometricJacobian(puma, config + h * currentJointVel, eeName);
        Jminus = geometricJacobian(puma, config - h * currentJointVel, eeName);
        Jdot = (Jplus - Jminus) / (2 * h);

        % Compute joint accelerations using inverse Jacobian relation:
        % eeAcc = J*qdd + Jdot*qd  =>  qdd = J_inv * (eeAcc - Jdot * qd)
        qdd(:, idx) = J_inv * (eeAcc(:, idx) - Jdot * qd(:, idx));

        % Zero out qdd for the same index where qd was below velThreshold
        qdd(zeroVelMask, idx) = 0;
    end

    % Final threshold filtering to ensure all elements satisfy the cutoff
    zeroVelAll = abs(qd) < velThreshold;
    qd(zeroVelAll) = 0;
    qdd(zeroVelAll) = 0;
end
