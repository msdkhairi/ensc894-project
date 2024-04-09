clear all; close all; clc;

% Define the file names
fileNames = {'planned_positions.csv', 'executed_positions.csv', 'planned_trajectory.csv', 'executed_trajectory.csv', 'velocities.csv', 'accelerations.csv'};

% Create the 3x2 subplot
figure;
% Set the figure size [x y width height]
set(gcf, 'Position', [100, 100, 1524, 768]); % Example: Creates a figure 1024x768 pixels

while true
    % Initialize a flag to check file existence
    filesExist = true;
    
    % Check each file individually
    for i = 1:length(fileNames)
        if ~isfile(fileNames{i})
            filesExist = false;
            break; % Exit the loop if a file is missing
        end
    end

    pause(0.1);


    %if all(cellfun(@(f) isfile(f), fileNames))
    if filesExist

        % Read the CSV files
        planned_positions_data = readtable('planned_positions.csv');
        executed_positions_data = readtable('executed_positions.csv');
        planned_trajectory_data = readtable('planned_trajectory.csv');
        executed_trajectory_data = readtable('executed_trajectory.csv');
        velocities_data = readtable('velocities.csv');
        accelerations_data = readtable('accelerations.csv');
        
        % Extract data for trajectory
        time_traj = planned_trajectory_data.time;
        theta1_traj = planned_trajectory_data.pos_theta1;
        theta2_traj = planned_trajectory_data.pos_theta2;
        theta3_traj = planned_trajectory_data.pos_d3;
        theta4_traj = planned_trajectory_data.pos_theta4;

        % Extract data for trajectory
        time_traj_e = executed_trajectory_data.time;
        theta1_traj_e = executed_trajectory_data.pos_theta1;
        theta2_traj_e = executed_trajectory_data.pos_theta2;
        theta3_traj_e = executed_trajectory_data.pos_d3;
        theta4_traj_e = executed_trajectory_data.pos_theta4;
        
        % Extract data for positions
        time_pos = planned_positions_data.time;
        pos_x = planned_positions_data.pos_x;
        pos_y = planned_positions_data.pos_y;

        % Extract data for positions
        time_pos_e = executed_positions_data.time;
        pos_x_e = executed_positions_data.pos_x;
        pos_y_e = executed_positions_data.pos_y;
        
        % Extract data for velocities
        time_vel = velocities_data.time;
        theta1_vel = velocities_data.vel_theta1;
        theta2_vel = velocities_data.vel_theta2;
        theta3_vel = velocities_data.vel_d3;
        theta4_vel = velocities_data.vel_theta4;
        
        % Extract data for accelerations
        time_acc = accelerations_data.time;
        theta1_acc = accelerations_data.acc_theta1;
        theta2_acc = accelerations_data.acc_theta2;
        theta3_acc = accelerations_data.acc_d3;
        theta4_acc = accelerations_data.acc_theta4;
    
    
    
        % Subplot for positions (now with pos_x and pos_y)
        subplot(2, 3, 1);
        plot(pos_x, pos_y, 'm'); % Plot positions in magenta
        xlim([0 400]);
        xlabel('Position X');
        ylabel('Position Y');
        title('Planned Position Plot');

        % Subplot for positions (now with pos_x and pos_y)
        subplot(2, 3, 4);
        plot(pos_x_e, pos_y_e, 'm'); % Plot positions in magenta
        xlim([0 400]);
        xlabel('Position X');
        ylabel('Position Y');
        title('Executed Position Plot');

        % Subplot for trajectory (now with theta1 to theta4)
        subplot(2, 3, 2);
        cla;
        hold on;
        plot(time_traj, theta1_traj, 'r');
        plot(time_traj, theta2_traj, 'g');
        plot(time_traj, theta3_traj, 'b');
        plot(time_traj, theta4_traj, 'k');
        xlabel('Time (s)');
        ylabel('Trajectory Theta Values (deg)');
        title('Planned Trajectory Thetas Over Time');
        legend('Theta1', 'Theta2', 'd3', 'Theta4');
        hold off;
        
        % Subplot for trajectory (now with theta1 to theta4)
        subplot(2, 3, 3);
        cla;
        hold on;
        plot(time_traj_e, theta1_traj_e, 'r');
        plot(time_traj_e, theta2_traj_e, 'g');
        plot(time_traj_e, theta3_traj_e, 'b');
        plot(time_traj_e, theta4_traj_e, 'k');
        xlabel('Time (s)');
        ylabel('Trajectory Theta Values (deg)');
        title('Executed Trajectory Thetas Over Time');
        legend('Theta1', 'Theta2', 'd3', 'Theta4');
        hold off;
        
        % Subplot for velocities
        subplot(2, 3, 5);
        cla;
        hold on;
        plot(time_vel, theta1_vel, 'r--');
        plot(time_vel, theta2_vel, 'g--');
        plot(time_vel, theta3_vel, 'b--');
        plot(time_vel, theta4_vel, 'k--');
        xlabel('Time (s)');
        ylabel('Velocity Theta Values (deg/s)');
        title('Velocity Thetas Over Time');
        legend('Theta1', 'Theta2', 'd3', 'Theta4');
        hold off;
        
        % Subplot for accelerations
        subplot(2, 3, 6);
        cla;
        hold on;
        plot(time_acc, theta1_acc, 'r:');
        plot(time_acc, theta2_acc, 'g:');
        plot(time_acc, theta3_acc, 'b:');
        plot(time_acc, theta4_acc, 'k:');
        xlabel('Time (s)');
        ylabel('Acceleration Theta Values (deg/(s*s))');
        title('Acceleration Thetas Over Time');
        legend('Theta1', 'Theta2', 'd3', 'Theta4');
        hold off;
        pause(0.1);
    else
        % If any file does not exist, wait for a second and check again
        pause(0.2);
    end
end