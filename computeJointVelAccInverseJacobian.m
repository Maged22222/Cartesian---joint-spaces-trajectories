function [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName, lambda, condThreshold)
% COMPUTEJOINTVELACCINVERSEJACOBIAN Computes joint velocities (qd) and 
% accelerations (qdd) from task-space velocity and acceleration trajectories 
% using Damped Least Squares (DLS) / pseudo-inverse of the geometric Jacobian.
%
% Syntax:
%   [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName)
%   [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName, lambda)
%   [qd, qdd] = computeJointVelAccInverseJacobian(puma, q, trajTimes, eeVel, eeAcc, eeName, lambda, condThreshold)
%
% Inputs:
%   puma          - RigidBodyTree robot model (e.g., puma560)
%   q             - numJoints x numSteps matrix of joint positions
%   trajTimes     - 1 x numSteps vector of trajectory timestamps
%   eeVel         - 6 x numSteps (or 3 x numSteps) task-space velocity [angular; linear]
%   eeAcc         - 6 x numSteps (or 3 x numSteps) task-space acceleration [angular; linear]
%   eeName        - End-effector body name (e.g., 'link7')
%   lambda        - (Optional) DLS damping factor for singularity robustness (default: 0.05).
%                   Set lambda = 0 for standard un-damped pseudo-inverse.
%   condThreshold - (Optional) Singularity condition number threshold (default: 1e6)
%
% Outputs:
%   qd            - numJoints x numSteps matrix of joint velocities
%   qdd           - numJoints x numSteps matrix of joint accelerations

    if nargin < 7 || isempty(lambda)
        lambda = 0.05; % Default DLS damping factor
    end

    if nargin < 8 || isempty(condThreshold)
        condThreshold = 1e6;
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

    for idx = 1:numSteps
        % Get joint configuration at this time step
        config = q(:, idx)';

        % Compute Jacobian at the current configuration
        J = geometricJacobian(puma, config, eeName);

        % Compute Jacobian inverse using Damped Least Squares (DLS) or pinv
        if lambda > 0
            % SVD-based Damped Least Squares: J^* = V * diag(s / (s^2 + lambda^2)) * U'
            [U, S, V] = svd(J, 'econ');
            s = diag(S);
            s_dls = s ./ (s.^2 + lambda^2);
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
    end
end
