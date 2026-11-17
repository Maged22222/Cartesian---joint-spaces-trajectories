function plotJointComparison(trajTimes, q1, q2, waypointTimes, labels, numJoints)

    numWaypoints = numel(waypointTimes);

    for i = 1:numJoints
        figure;
        hold on;
        plot(trajTimes, q1(i, :), 'LineWidth', 1.5);
        plot(trajTimes, q2(i, :), 'LineWidth', 1.5);

        for wIdx = 1:numWaypoints
            xline(waypointTimes(wIdx), 'k--', 'HandleVisibility', 'off');
        end

        title(['Joint ', num2str(i), ' Position']);
        xlabel('Time (s)');
        ylabel('Position (rad)');
        legend(labels{1}, labels{2});
        grid on;
    end
end
