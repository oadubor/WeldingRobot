classdef RobotPlanning3D 
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'public')                        
        robot % robot object
        straight_waypoint_data % workspace trajectories 
        orientation_ik_weight % weight to modify ik orientation value
        yaw_orientation_ik_weight
        pitch_orientation_ik_weight
        spline_time % for spline
        straight_trajectory_spline_time
        frequency % for spline
    end
    
    methods
        % Constructor 
        function planning = RobotPlanning3D(robot)
            planning.robot = robot;
            straight_waypoint_file = 'straight.csv';
            planning.straight_waypoint_data = csvread(straight_waypoint_file); % straight csv
            planning.spline_time = 0.2; % seconds 
            planning.straight_trajectory_spline_time = 0.2;
            planning.frequency = 100; % Hz
            
            %planning.orientation_ik_weight = 0.00085; 
            planning.yaw_orientation_ik_weight = 0.001;
            %planning.yaw_orientation_ik_weight = 0.05;
            %planning.pitch_orientation_ik_weight = 0.0011;
            planning.pitch_orientation_ik_weight = 0.05;
            %planning.orientation_ik_weight = 1; 
        end
        
        function [] = save_trajectory_file(~, trajectory, filename)
            % Save trajectory to csv file                        
            trajectory_folder_path = pwd + ...
                "/Fall 18/Kinematics & Dynamics/Capstone/Code Handout /handout/trajectories/";
            trajectory_filename = trajectory_folder_path + filename;
            csvwrite(trajectory_filename, trajectory);         
        end
        
        function c = cost(planning,thetas, goal_position) % IK Cost Function                 
            ee = planning.robot.ee(thetas);
            % weight to change cost of orientation values
            yaw_orientation_weight = planning.yaw_orientation_ik_weight; 
            pitch_orientation_weight = planning.pitch_orientation_ik_weight;
            ee_focus = [ee(1);ee(2);ee(3);ee(4);ee(5)];
            %goal_position
            pose_delta = ee_focus-goal_position(1:5); % current_pos - goal_pos
            % Change weight of orientations
            pose_delta(4) = pose_delta(4)*yaw_orientation_weight;            
            pose_delta(5) = pose_delta(5)*pitch_orientation_weight;
            % get scalar cost value
            c = sum((pose_delta).^2); 
        end
        
        function [ trajectory ] = trajectory_spline(~,waypoints, times, frequency)
            % trajectory_const_vel
            %
            %   Returns a matrix of joint angles, where each column represents a single
            %   timestamp. These joint angles form constant velocity trajectory segments,
            %   hitting waypoints(:,i) at times(i).
            %
            %   'waypoints' is a matrix of waypoints; each column represents a single
            %   waypoint in joint space, and each row represents a particular joint.
            %
            %   'times' is a row vector that indicates the time each of the waypoints should
            %   be reached.  The number of columns should equal the number of waypoints, and
            %   should be monotonically increasing.  The first number is technically
            %   arbitrary, as it serves only as a reference for the remaining timestamps,
            %   but '0' is a good default.
            %
            %   'frequency' is the control frequency which this trajectory should be played
            %   at, and therefore the number of columns per second of playback.

            % Number of joints:
            num_joints = size(waypoints, 1);
            % Number of waypoints:
            num_waypoints = size(waypoints, 2);
            % Number of segments between waypoints:
            num_segments = num_waypoints - 1;

            if (size(times) ~= [1, num_waypoints])
                error('Size of times vector is incorrect!');
            end

            if (num_waypoints < 2)
                error('Insufficient number of waypoints.');
            end

            if (length(frequency) ~= 1 || frequency < 5)
                error('Invalid control frequency (must be at least 5Hz)');
            end

            % First, we go through each segment between waypoints, and calculate how many
            % points we must generate in this segment given our control frequency.
            num_points_per_segment = zeros(1, num_segments);
            for segment = 1:num_segments
                dt = times(segment+1) - times(segment);
                num_points_per_segment(segment) = dt * frequency;
            end

            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Compute the cubic spline which interpolates the given waypoints, and has zero
            % velocity at each endpoint.
            %
            % A spline can be directly filled in all at once, by using the 'spline' function.
            %
            % Example: given 't' and 'y' (values of unknown function at 't'):
            %   yy = spline(x, y, xx) 
            % returns interpolated values 'yy' at each point 'xx'.
            %
            % Note: spline(x, [0 y 0], xx) constrains the first/last point to have% zero
            % velocity. Here, '0' must be sized appropriately for the number of rows in 'y'.            
            trajectory = zeros(num_joints, sum(num_points_per_segment)); % Replace this with actual value of trajectory!
            for joint = 1:num_joints
                % zero velocity waypoints array
                waypoints_prime = [0 waypoints(joint,:) 0];
                waypoint_times = times(1:end); % waypoint time indices
                interpolated_times = linspace(times(1),times(end), ...
                    sum(num_points_per_segment)); 
                % Create paths 
                trajectory(joint,:) = spline(waypoint_times, waypoints_prime, ...
                    interpolated_times);
            end            
        end
        
        function thetas = inverse_kinematics(planning, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= planning.robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', ...
                    size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 6 && size(goal_position, 2) ~= 1)
                error('Invalid goal_position: Should be a 6 length column vector, is %dx%d.', ...
                    size(goal_position, 1), size(goal_position, 2));
            end
            
            options = optimoptions(@fmincon,'Algorithm','sqp');
            thetas = fmincon(@(thetas) planning.cost(thetas,goal_position), ...
                initial_thetas,[],[],[],[],planning.robot.LB,planning.robot.UB,[],options);
        
% --------------- END STUDENT SECTION ------------------------------------
        end
        
        % Shorthand for returning the end effector position and orientation of IK solution 
        function ik = ik(planning, initial_thetas, goal_position)
            ik = planning.inverse_kinematics(initial_thetas, goal_position);
        end
        
        function [trajectory] = create_straight_trajectory(planning,initial_thetas, ...
                insert_thetas, vars, full_path, approach_resolution, ...
                trajectory_resolution, x_trajectory_offset, ...
                y_trajectory_offset,z_trajectory_offset, save_file)
            % Creates trajectory to move the robot in front of the straight 
            % path and then an additional trajectory to follow the path            
            % trajectory
            % inputs:
            %   initial_thetas = initial joint config
            %   goal_position = desired end position
            %   vars = number of workspace variables to account for in ik
            %   full_path (bool) = creates path for entire straight path
            %   approach_resolution = num points for initial approach
            %   trajectory_resolution = num points for path trajectory
            %   expand_waypoints (bool) = increase resolution for straight path
            %   save_file (bool) = save joint trajectory as csv            
            if size(insert_thetas, 2) ~= 1
                error('Expecting a workspace position column vector.');
            end
            if approach_resolution < 1 || trajectory_resolution < 1
                error('trajectory resolutions must be at least 1');
            end
                        
            % Spline joint space trajectory for moving to home position
            spline_waypoints = horzcat(initial_thetas, insert_thetas);
            times = [0 planning.spline_time];            
            home_trajectory = planning.trajectory_spline(spline_waypoints, ...
                times, planning.frequency); % column vector                         
            waypoint_thetas = home_trajectory(:,end); % theta of robot waypoint position
            waypoint_position = planning.robot.ee(waypoint_thetas); % workspace
            initial_move_joint_trajectory = home_trajectory;       
            
            % Create trajectory for straight path 
            if full_path                 
                % Create trajectory offset
                trajectory_offset_array = zeros(length(planning.straight_waypoint_data), ...
                    size(planning.straight_waypoint_data, 2));
                % trajectory offsets
                trajectory_offset_array(:,1) = x_trajectory_offset;
                trajectory_offset_array(:,2) = y_trajectory_offset;
                trajectory_offset_array(:,3) = z_trajectory_offset;  
                % add trajectory offsets to array
                workspace_trajectory = planning.straight_waypoint_data + trajectory_offset_array;
                
                % create linear workspace trajectory of waypoint_position
                % to first trajectory_index, i.e. the approach
                approach_workspace_trajectory = zeros(approach_resolution, 3);
                for i = 1 : size(workspace_trajectory,2)
                    approach_workspace_trajectory(:,i) = linspace(waypoint_position(i), ...
                        workspace_trajectory(1,i), approach_resolution);
                end
                                
                % combine approach trajectory with straight trajectory
                workspace_trajectory = vertcat(approach_workspace_trajectory, workspace_trajectory);                                
                % get length of waypoint trajectory
                num_trajectory_rows = length(workspace_trajectory);                                
                % create joint trajectory (thetas)
                goal_pos = zeros(vars,1); % store workspace variables
                straight_joint_trajectory = zeros(num_trajectory_rows,planning.robot.dof); 
                % loop through waypoints & perform IK      
                initial_thetas = insert_thetas;
                %straight_joint_trajectory(1,:) = insert_thetas;
                for  i = 1 : num_trajectory_rows
                    goal_pos(1:3) = workspace_trajectory(i,:)'; % [x;y;z]
                    goal_pos(4,1) = pi/2; % tune for pitch about y
                    goal_pos(5,1) = 0; % tune for yaw about x                     
                    ik_thetas = planning.ik(initial_thetas, goal_pos);                      
                    straight_joint_trajectory(i,:) = ik_thetas;                    
                    % reset arguments for next IK iteration
                    initial_thetas = ik_thetas;                     
                end                        
                % trajectory without spline expansion
                trajectory = [transpose(initial_move_joint_trajectory); straight_joint_trajectory];
                % Create spline of approach & straight trajectory
                times = [0 planning.straight_trajectory_spline_time];  
                num_spline_segments = planning.straight_trajectory_spline_time*planning.frequency;
                num_spline_trajectory_rows = num_spline_segments * length(trajectory);
                splined_trajectory = zeros(num_spline_trajectory_rows-num_spline_segments, 5);
                index = 1; 
                for i = 1 : length(trajectory) - 1                    
                    % grab 2 waypoints from trajectory
                    spline_waypoint = horzcat(trajectory(i,:)', trajectory(i+1,:)');                     
                    splined_trajectory(index:(index+num_spline_segments-1), :) = planning.trajectory_spline(...
                        spline_waypoint, times, planning.frequency)'; % column vector 
                    index = index + num_spline_segments; % increment index                    
                end
                trajectory = splined_trajectory; % return trajectory
                
                %return
                                          
                %trajectory = planning.trajectory_spline(trajectory', ...
                %    times, planning.frequency); % column vector              
                %return
                %Increase Resolution of trajectory for straight path
                %{
                if expand_waypoints       
                    num_rows = num_trajectory_rows*trajectory_resolution; 
                    expanded_waypoint_trajectory = zeros(...
                        num_rows, planning.robot.dof);
                    res = trajectory_resolution; % resolution                                                                                               
                    index = 1; % keep track of place in array                    
                    for i = 1 : length(trajectory)-1  
                        initial_points = trajectory(i,:);
                        end_points = trajectory(i+1,:);
                        % avoid putting in duplicate end points
                        end_points = end_points - (end_points - initial_points)/res;
                        % expand each joint's theta trajectory
                        for j = 1 : planning.robot.dof                            
                            expanded_waypoint_trajectory(index:index+res-1,j) = linspace(...
                                initial_points(j), end_points(j), res);
                        end                             
                        index = index + res; % calculate new index for array access                          
                    end                    
                    trajectory = expanded_waypoint_trajectory;   
                  
                end
                %}

            end
            
            if save_file
                planning.save_trajectory_file(trajectory,"my_straight_trajectory.csv");
            end
           
                                                         
        end
        
    
    end
end
