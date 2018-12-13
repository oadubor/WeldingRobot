%% 16-384 Kinematics & Dynamics - Obi Adubor 
clear all;
clc;
close all;

robot = Robot3D();
planning = RobotPlanning3D(robot);
%frames = robot.jacobians(initial_thetas);
%% Waypoint Trajectory Files
% Waypoint File Names
straight_waypoint_file = 'straight.csv';
sine_waypoint_file = 'sine.csv';
% store file data
straight_waypoint_data = csvread(straight_waypoint_file); % straight csv
sine_waypoint_data = csvread(sine_waypoint_file); % sine csv

%% IK Testing
%{
% straight path
straight_ee_positions = zeros(size(straight_waypoint_data,1), ...
    size(straight_waypoint_data,2));
% sine path
sine_ee_positions = zeros(size(sine_waypoint_data,1), ...
    size(sine_waypoint_data,2));
goal_pos = zeros(5,1);  % orientation is insignificant (right now) - roll matters
initial_thetas = robot.straight_initial_thetas;
%%%%%%%%%%%%%%%%%%
% Straight Path IK
%%%%%%%%%%%%%%%%%%
initial_pos = robot.ee(robot.straight_initial_thetas);
% IK for linear workspace trajectory into straight path
initial_move_resolution = 50;
initial_move_positions = zeros(initial_move_resolution,5);
initial_move_ee_positions = zeros(initial_move_resolution,5);
initial_move_joint_trajectory = zeros(initial_move_resolution,5);
%straight_waypoint_data(1,i)

for i = 1:size(initial_move_positions,2)-2
    initial_move_positions(:,i) = linspace(initial_pos(i), ...
        straight_waypoint_data(1,i), initial_move_resolution);
end
%straight_waypoint_data
for  i = 1:size(initial_move_positions,1)
    goal_pos(1:5,:) = initial_move_positions(i,:)';            
    ik_thetas = planning.ik(initial_thetas, goal_pos); %% ee position from ik
    %end    
    initial_move_joint_trajectory(i,:) = ik_thetas;
    ee = robot.ee(ik_thetas);
    initial_move_ee_positions(i,1) = ee(1);
    initial_move_ee_positions(i,2) = ee(2);
    initial_move_ee_positions(i,3) = ee(3);
    initial_move_ee_positions(i,4) = ee(4);
    % reset arguments for IK
    initial_thetas = ik_thetas;
    %initial_pos = ee; 
end
% Create x-y plot
figure;
hold on
plot(initial_move_ee_positions(:,1),initial_move_ee_positions(:,2));
xlabel('x-axis');
ylabel('y-axis');
title('xy plane');
grid on 
hold off
% Create x, y, z plot
figure;
subplot(3,1,1);
hold on
plot(initial_move_ee_positions(:,1));
title('x-axis');
grid on
hold off
subplot(3,1,2);
grid on
hold on
plot(initial_move_ee_positions(:,2));
title('y-axis');
grid on
subplot(3,1,3);
hold on
plot(initial_move_ee_positions(:,3));
title('z-axis');
grid on
% Create 3D Plot
figure;
plot3(initial_move_ee_positions(:,1),initial_move_ee_positions(:,2),initial_move_ee_positions(:,3));
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
title('IK Solution')
grid on
%initial_move_positions
%}
%% IK Straight Path Trajectory Testing
initial_thetas = [0;0;0;0;0];
%y_trajectory_offset = 
x_approach_offset = -0.02+0.02;
y_approach_offset = -0.02+0.02; % for approach, makes robot insert into path slowly
z_approach_offset = 0.01-0.01;
setup_position = straight_waypoint_data(1,:)' + [x_approach_offset; y_approach_offset; z_approach_offset]; % modify approach
%setup_position = [0.1800;0.5;0.27];
setup_position(4,1) = 1.68; % tune for pitch about y
setup_position(5,1) = 0; % tune for yaw about x    
approach_resolution = 30;
trajectory_resolution =3;
full_path = true;
save_file = true;
expand_waypoints = true;
x_trajectory_offset = 0.02;
y_trajectory_offset = -0.02;
z_trajectory_offset = -0.06;


setup_position = [0.2994; 0.8649; 1.0318; -0.1222; 0.4180];


trajectory = planning.create_straight_trajectory(initial_thetas, ...
                    setup_position, 5, full_path, approach_resolution, ...
                    trajectory_resolution, ...
                    x_trajectory_offset, y_trajectory_offset, z_trajectory_offset,save_file);                

straight_ee_positions = zeros(length(trajectory),3);
for i = 1:length(trajectory)
    ee = robot.ee(trajectory(i,:)');
    straight_ee_positions(i,1) = ee(1);
    straight_ee_positions(i,2) = ee(2);
    straight_ee_positions(i,3) = ee(3);    
end
% Create x-y plot
figure;
hold on
plot(straight_ee_positions(:,1),straight_ee_positions(:,2));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
title('xy plane');
grid on 
hold off
% Create x, y, z plot
figure;
subplot(3,1,1);
hold on
plot(straight_ee_positions(:,1));
title('x-axis [m]');
grid on
hold off
subplot(3,1,2);
grid on
hold on
plot(straight_ee_positions(:,2));
title('y-axis [m]');
grid on
subplot(3,1,3);
hold on
plot(straight_ee_positions(:,3));
title('z-axis [m]');
grid on
% Create 3D Plot
figure;
plot3(straight_ee_positions(:,1),straight_ee_positions(:,2),straight_ee_positions(:,3));
xlabel('x-axis [m]');
ylabel('y-axis [m]');
zlabel('z-axis [m]');
title('IK Solution')
grid on
%initial_move_positions



%{
%straight_ee_positions(1,:) = initial_pos(1:3);
for  i = 1:size(straight_waypoint_data,1)
    goal_pos(1:3,:) = straight_waypoint_data(i,:)';            
    ik_thetas = robot.ik(initial_thetas, goal_pos); %% ee position from ik
    %end    
    ee = robot.ee(ik_thetas);
    straight_ee_positions(i,1) = ee(1);
    straight_ee_positions(i,2) = ee(2);
    straight_ee_positions(i,3) = ee(3);
    straight_ee_positions(i,4) = ee(4);
    % reset arguments for IK
    initial_thetas = ik_thetas;
    %initial_pos = ee; 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot fk ee position from waypoint file vs. ik ee pos
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
hold on
%plot3(straight_ee_positions(1,1),straight_ee_positions(1,2),straight_ee_positions(1,3),'r*');
plot3(straight_ee_positions(:,1), straight_ee_positions(:,2), straight_ee_positions(:,3));
%plot3(straight_waypoint_data(:,1), straight_waypoint_data(:,2), straight_waypoint_data(:,3));
legend('IK solution');
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
grid on
hold off
figure;
plot(straight_ee_positions(:,4));
title('Yaw');
ylabel('Radians');
% Plot x positions
%}


%{
title('FK ee positions vs. IK ee positions');
subplot(1,3,1);
hold on
plot(straight_ee_positions(:,1),'k');
plot(waypoint_data(:,1),'g--');
title('x positions');
ylabel('Position (mm)')
grid on
% Plot y positions
subplot(1,3,2);
hold on
plot(straight_ee_positions(:,2),'k');
plot(waypoint_data(:,2),'g--');
title('y positions');
xlabel('green=theirs, black=yours');
grid on
% Plot z positions
subplot(1,3,3);
hold on
plot(straight_ee_positions(:,3),'k');
plot(waypoint_data(:,3),'g--');
title('z positions');
grid on
%}
