function plots(trajTimes, eePos, eeOrient, eeVel, eeAcc, qdd, numJoints, dt, waypoints, title3D)


    tPos = trajTimes(1:size(eePos, 2));
    tOrient = trajTimes(1:size(eeOrient, 2));
    tVel = trajTimes(1:size(eeVel, 2));
    tAcc = trajTimes(1:size(eeAcc, 2));

    if size(eeAcc, 2) > 1
        eeJerk = diff(eeAcc, 1, 2) / dt;
        tJerk = trajTimes(1:size(eeJerk, 2));
    else
        eeJerk = zeros(size(eeAcc));
        tJerk = tAcc;
    end

    figure;

    % Position
    subplot(3, 1, 1);
    plot(tPos, eePos(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tPos, eePos(2, :), 'g', 'LineWidth', 1.5);
    plot(tPos, eePos(3, :), 'b', 'LineWidth', 1.5);
    title(['Task-Space Position',title3D]);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X', 'Y', 'Z');
    grid on;

    % Plot orientation (roll, pitch, yaw)
    subplot(3, 1, 2);
    plot(tOrient, eeOrient(1, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tOrient, eeOrient(2, :), 'g', 'LineWidth', 1.5);
    plot(tOrient, eeOrient(3, :), 'b', 'LineWidth', 1.5);
    title(['Task-Space Orientation (Roll, Pitch, Yaw)',title3D]);
    xlabel('Time (s)');
    ylabel('Orientation (rad)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;

    % Plot velocity (linear + angular)
    subplot(3, 1, 3);
    plot(tVel, eeVel(4, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tVel, eeVel(5, :), 'g', 'LineWidth', 1.5);
    plot(tVel, eeVel(6, :), 'b', 'LineWidth', 1.5);
    plot(tVel, eeVel(1, :), 'c--', 'LineWidth', 1.5); 
    plot(tVel, eeVel(2, :), 'm--', 'LineWidth', 1.5); 
    plot(tVel, eeVel(3, :), 'k--', 'LineWidth', 1.5); 
    title(['Task-Space Velocity (Linear + Angular)',title3D]);
    xlabel('Time (s)');
    ylabel('Velocity');
    legend('Vx', 'Vy', 'Vz', 'Wx', 'Wy', 'Wz');
    grid on;

    %  Acceleration (Linear + Angular)
    figure;
    plot(tAcc, eeAcc(4, :), 'r', 'LineWidth', 1.5); hold on;
    plot(tAcc, eeAcc(5, :), 'g', 'LineWidth', 1.5);
    plot(tAcc, eeAcc(6, :), 'b', 'LineWidth', 1.5); 
    plot(tAcc, eeAcc(1, :), 'c--', 'LineWidth', 1.5); 
    plot(tAcc, eeAcc(2, :), 'm--', 'LineWidth', 1.5); 
    plot(tAcc, eeAcc(3, :), 'k--', 'LineWidth', 1.5); 
    title(['Task-Space Acceleration (Linear + Angular)',title3D]);
    xlabel('Time (s)');
    ylabel('Acceleration');
    legend('Ax', 'Ay', 'Az', 'AlphaX', 'AlphaY', 'AlphaZ');
    grid on;

    % Jerks (Linear + Angular)
    figure;
    plot(tJerk, eeJerk(4, :), 'r', 'LineWidth', 1.5); hold on; 
    plot(tJerk, eeJerk(5, :), 'g', 'LineWidth', 1.5);
    plot(tJerk, eeJerk(6, :), 'b', 'LineWidth', 1.5); 
    plot(tJerk, eeJerk(1, :), 'c--', 'LineWidth', 1.5); 
    plot(tJerk, eeJerk(2, :), 'm--', 'LineWidth', 1.5); 
    plot(tJerk, eeJerk(3, :), 'k--', 'LineWidth', 1.5); 
    title(['Task-Space Jerk (Linear + Angular)',title3D]);
    xlabel('Time (s)');
    ylabel('Jerk');
    legend('Jx', 'Jy', 'Jz', 'AlphaDotX', 'AlphaDotY', 'AlphaDotZ');
    grid on;

    % 3D Task-Space Trajectory
    if nargin >= 9 && ~isempty(waypoints)
        figure;
        plot3(eePos(1, :), eePos(2, :), eePos(3, :), 'b-', 'LineWidth', 1.5);
        hold on;
        plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'MarkerSize', 6);
        
        title(['End-Effector Task-Space Trajectory',title3D]);
        
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        grid on;
        legend('Trajectory', 'Waypoints');
    end
end
