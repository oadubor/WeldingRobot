%% 16-384 Kinematics & Dynamics - Obi Adubor 
clear all;
clc;
close all;
%% 5-DOF DH Parameters (Initial)
% Link Lengths in m
a = 116.23/1000;
b = 91.05/1000;
c = 327.76/1000;
d = 91.05/1000;
e = 94.05/1000;
f = 279.5/1000;
g = 57.4/1000;
h = 266.7/1000;
initial_thetas = [0;0;0;0;0];
% Store dh parameters
dh_parameters = [0, pi/2, a, 0;
                 c, pi, b, 0;
                 0, pi/2, d, pi/2;
                 f, pi/2, e, pi/2;
                 h, pi/2, g, 0];
                 %0, 0, h, 0];
LB = [-pi/6; deg2rad(-15); -2*pi; -pi; -pi/6]; % lower bound for IK
UB = [deg2rad(200); deg2rad(200); 2*pi; pi; pi/6]; % upper bound for IK
 %% Create Robot
robot = Robot3D();
%frames = robot.jacobians(initial_thetas);
%% Waypoint Trajectory Files
% Waypoint File Names
waypoint_file = 'waypoints.csv';
waypoint_joint_file = 'waypoints_joints.csv';
% Get size of files
waypoint_data = csvread(waypoint_file);
waypoint_joint_data = csvread(waypoint_joint_file);
[waypoint_file_rows, waypoint_file_cols] = size(waypoint_data);
[waypoint_joint_file_rows, waypoint_joint_file_cols] = size(waypoint_joint_data);
%% Waypoint Trajectory Testing

% Store fk ee positions
fk_ee_positions = zeros(waypoint_joint_file_rows, waypoint_file_cols);

% Map ee position from given thetas
for i = 1:waypoint_joint_file_rows
%for i = 1:1
    thetas = waypoint_joint_data(i,:)'; % get thetas from file 
    ee = robot.ee(thetas); %% ee position
    J = robot.jacobians(thetas);
    fk_ee_positions(i,1) = ee(1);
    fk_ee_positions(i,2) = ee(2);
    fk_ee_positions(i,3) = ee(3);
end
%{
control = RobotControl3D(robot, 'Robot A');
control.fix_hebilookup();
control.gains();
control.follow_trajectory(waypoint_joint_data);
%}
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ee_position vs. waypoints file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% Plot x positions
title('FK ee positions vs. given waypoint ee positions');
subplot(1,3,1);
hold on
plot(waypoint_data(:,1),'g--');
plot(ee_positions(:,1),'k');
title('x positions');
ylabel('Position (mm)')
grid on
% Plot y positions
subplot(1,3,2);
hold on
plot(waypoint_data(:,2),'g--');
plot(ee_positions(:,2),'k');
title('y positions');
xlabel('green=theirs, black=yours');
grid on
% Plot z positions
subplot(1,3,3);
hold on
plot(waypoint_data(:,3),'g--');
plot(ee_positions(:,3),'k');
title('z positions');
grid on
%}
%{
%% IK Testing
ik_ee_positions = zeros(waypoint_joint_file_rows, waypoint_file_cols);
goal_pos = zeros(6,1);  % orientation is insignificant (right now)
original_thetas = waypoint_joint_data(1,:)'; % get thetas from file
initial_pos = robot.ee(initial_thetas);
ik_ee_positions(1,:) = initial_pos(1:3);
for  i = 1:waypoint_joint_file_rows  
    goal_pos(1:3,:) = waypoint_data(i,:)';            
    ik_thetas = robot.ik(initial_thetas, goal_pos); %% ee position from ik
    %end    
    ee = robot.ee(ik_thetas);
    ik_ee_positions(i,1) = ee(1);
    ik_ee_positions(i,2) = ee(2);
    ik_ee_positions(i,3) = ee(3);
    % reset arguments for IK
    initial_thetas = ik_thetas;
    %initial_pos = ee; 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot fk ee position from waypoint file vs. ik ee pos
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% Plot x positions
title('FK ee positions vs. IK ee positions');
subplot(1,3,1);
hold on
plot(ik_ee_positions(:,1),'k');
plot(waypoint_data(:,1),'g--');
title('x positions');
ylabel('Position (mm)')
grid on
% Plot y positions
subplot(1,3,2);
hold on
plot(ik_ee_positions(:,2),'k');
plot(waypoint_data(:,2),'g--');
title('y positions');
xlabel('green=theirs, black=yours');
grid on
% Plot z positions
subplot(1,3,3);
hold on
plot(ik_ee_positions(:,3),'k');
plot(waypoint_data(:,3),'g--');
title('z positions');
grid on
%}