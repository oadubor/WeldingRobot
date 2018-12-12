function [] = analyze_hebilog_data
    close all;
    clc;
    % Analyze thetas, velocities, torques
    currentDir = fileparts(mfilename('fullpath'));
    hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));    
    % Plot angle data
    figure();
    title('Plot of joint positions during trajectory');
    for i = 1:5
        subplot(5,1,i);
        hold on
        grid on
        plot(hebilog.time, hebilog.positionCmd(:,i));
        plot(hebilog.time, hebilog.position(:,i), '--');  
        xlabel("Theta " + num2str(i));
        ylabel('\theta');
        hold off
        %legend('commanded', 'actual');
    end
   
    figure();
    subplot(2,1,1);
    plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
    hold on;
    plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
    hold off;
    title('Plot of joint velocities during trajectory');
    xlabel('t');
    ylabel('joint velocities');
    subplot(2,1,2);
    plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
    title('Plot of joint torques during trajectory');
    xlabel('t');
    ylabel('\tau');    
    
    % Analyze end effector position
    robot = Robot3D();
    ee_actual_positions = zeros(length(hebilog.position),3); % actual positions
    ee_cmd_positions = zeros(length(hebilog.position),3); % commanded positions
    ee_orientation_actual = zeros(length(hebilog.position),3);
    for i = 1:length(hebilog.position)
        ee_actual = robot.ee(hebilog.position(i,:)');
        ee_cmd = robot.ee(hebilog.positionCmd(i,:)');
        % Store actual positions
        ee_actual_positions(i,1:3) = ee_actual(1:3);        
        % Store actual orientation
        ee_orientation_actual(i,1:3) = ee_actual(4:6);
        % Store commanded positions
        ee_cmd_positions(i,1:3) = ee_cmd(1:3);                
    end
    % Create x-y plot of actual vs. command
    figure;
    hold on
    plot(ee_actual_positions(:,1),ee_actual_positions(:,2));
    plot(ee_cmd_positions(:,1),ee_cmd_positions(:,2));
    legend('actual', 'cmd');
    xlabel('x-axis [m]');
    ylabel('y-axis [m]');
    title('xy plane');
    grid on 
    hold off
    % Create x, y, z plot of actual vs. command
    figure;
    subplot(3,1,1);
    hold on
    plot(ee_actual_positions(:,1));
    plot(ee_cmd_positions(:,1));
    legend('actual', 'cmd');
    title('x-axis [m]');
    grid on
    hold off
    subplot(3,1,2);
    grid on
    hold on
    plot(ee_actual_positions(:,2));
    plot(ee_cmd_positions(:,2));
    legend('actual', 'cmd');
    title('y-axis [m]');
    grid on
    subplot(3,1,3);
    hold on
    plot(ee_actual_positions(:,3));
    plot(ee_cmd_positions(:,3));
    legend('actual', 'cmd');
    title('z-axis [m]');
    grid on
    
    % Create 3D Plot of actual vs. command
    figure;
    hold on 
    plot3(ee_actual_positions(:,1),ee_actual_positions(:,2),ee_actual_positions(:,3));    
    plot3(ee_cmd_positions(:,1),ee_cmd_positions(:,2),ee_cmd_positions(:,3));    
    legend('actual', 'commanded');
    xlabel('x-axis [m]');
    ylabel('y-axis [m]');
    zlabel('z-axis [m]');
    title('IK Solution');
    grid on
    
    % Create Plot of actual orientation
    figure;
    subplot(3,1,1);
    hold on
    title('End Effector Orientation');
    plot(ee_orientation_actual(:,1));    
    legend('Actual');
    xlabel('Pitch'); % actually pitch (not yaw) from ee perspective
    ylabel('Radians');    
    grid on
    hold off
    subplot(3,1,2);
    grid on
    hold on
    plot(ee_orientation_actual(:,2));    
    legend('Actual');
    ylabel('Radians');
    xlabel('Yaw'); % actually yaw (not pitch) from ee perspective
    grid on
    subplot(3,1,3);
    hold on
    plot(ee_orientation_actual(:,3));    
    legend('Actual');
    ylabel('Radians');
    xlabel('Roll');    
    grid on
    
    
end
