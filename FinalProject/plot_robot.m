function plot_robot(theta1, theta2)
    % Calculate the (x, y) position of the first joint
    L1 = 1.4;  % Length of the first link
    L2 = 1.4;  % Length of the second link
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    
    % Calculate the (x, y) position of the end effector (second joint)
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);
    
    % Plot the links of the robot
    plot([0, x1], [0, y1], 'ro-', 'LineWidth', 8);  % First link
    hold on;  % Hold the plot
    plot([x1, x2], [y1, y2], 'bo-', 'LineWidth', 8);  % Second link
    hold off;  % Release the hold
    
    % Set the plot limits
    xlim([-4, 4]);
    ylim([-4, 4]);
    
    % Set the aspect of the plot to be equal
    daspect([1,1,1]);
    
    % Add grid
    grid on;
    
    % Show the plot
    drawnow;
end