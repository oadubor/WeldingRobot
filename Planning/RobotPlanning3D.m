classdef RobotPlanning3D 
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'public')                
        %robot_joints
        %robot_name
        robot % robot object
        straight_waypoint_data
        %kp_gains
        %ki_gains
        %kd_gains
        %frequency
        %set_gains % boolean for setting gains
    end
    
    methods
        % Constructor 
        function planning = RobotPlanning3D(robot)
            planning.robot = robot;
            straight_waypoint_file = 'straight.csv';
            planning.straight_waypoint_data = csvread(straight_waypoint_file); % straight csv
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
            %weight_multiplier = 0.005; % weight to change cost of values
            weight_multiplier = 0.01; % weight to change cost of values
            ee_focus = [ee(1);ee(2);ee(3);ee(4);ee(5)];
            %goal_position
            pose_delta = ee_focus-goal_position(1:5); % current_pos - goal_pos
            % Change weight of orientations
            pose_delta(4) = pose_delta(4)*weight_multiplier;
            pose_delta(5) = pose_delta(5)*weight_multiplier;
            c = sum((pose_delta).^2); % ignores orientation
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
                goal_position, vars, full_path, approach_resolution, ...
                trajectory_resolution, expand_waypoints, save_file)
            % Creates trajectory to move the robot from current position 
            % to setup position where it can begin following a desired path
            % trajectory
            % inputs:
            %   resolution =  number of points to create
            if size(goal_position, 2) ~= 1
                error('Expecting a workspace position column vector.');
            end
            if approach_resolution < 1 || trajectory_resolution < 1
                error('trajectory resolutions must be at least 1');
            end
            goal_pos = zeros(vars,1); % [position; orientation]            
            initial_pos = planning.robot.ee(initial_thetas);
            % IK for linear workspace trajectory into straight path            
            initial_move_positions = zeros(approach_resolution,vars);
            %initial_move_ee_positions = zeros(resolution,vars);
            initial_move_joint_trajectory = zeros(approach_resolution,vars);
            % create linear x;y;z;yaw trajectory 
            for i = 1:size(initial_move_positions,2)-1
                initial_move_positions(:,i) = linspace(initial_pos(i), ...
                    goal_position(i,1), approach_resolution);
            end
            
            % run IK on initial movement for approach of trajectory
            for  i = 1:size(initial_move_positions,1)
                goal_pos(1:vars,:) = initial_move_positions(i,:)';            
                ik_thetas = planning.ik(initial_thetas, goal_pos); %% ee position from ik                
                initial_move_joint_trajectory(i,:) = ik_thetas;                
                initial_thetas = ik_thetas;                 
            end
            % trajectory for approach
            trajectory = initial_move_joint_trajectory;                        
            % Create trajectory for straight path
            if full_path
                % get length of waypoint file
                num_trajectory_rows = length(planning.straight_waypoint_data);                
                num_trajectory_cols = size(planning.straight_waypoint_data,2);
                if expand_waypoints % expand waypoint trajectories                    
                    %Increase Resolution of trajectory for straight path
                    expanded_waypoint_data = zeros(...
                        num_trajectory_rows*trajectory_resolution, num_trajectory_cols);
                    res = trajectory_resolution; % resolution                
                    %disp('hi')
                    index = 1; % keep track of place in array
                    for i = 1:num_trajectory_rows-1 
                        initial_points = planning.straight_waypoint_data(i,:);
                        end_points = planning.straight_waypoint_data(i+1,:);
                        % expand x
                        expanded_waypoint_data(index:index+res-1,1) = linspace(...
                            initial_points(1), end_points(1), res);
                        % expand y
                        expanded_waypoint_data(index:index+res-1,2) = linspace(...
                            initial_points(2), end_points(2), res);
                        % expand z
                        expanded_waypoint_data(index:index+res-1,3) = linspace(...
                            initial_points(3), end_points(3), res);
                        index = index + res; % calculate new index for array access                    
                    end     

                    straight_joint_trajectory = zeros(length(expanded_waypoint_data)-1,5); 
                    % loop through waypoints & perform IK
                    for  i = 1:length(straight_joint_trajectory)
                        goal_pos(1:3) = expanded_waypoint_data(i,:)';
                        goal_pos(4,1) = 1.68; % tune for pitch about y
                        goal_pos(5,1) = 0; % tune for yaw about x 
                        ik_thetas = planning.ik(initial_thetas, goal_pos);                      
                        straight_joint_trajectory(i,:) = ik_thetas;                    
                        % reset arguments for IK
                        initial_thetas = ik_thetas;                     
                    end                
                    % shave off excess zeros 
                    straight_joint_trajectory = straight_joint_trajectory(1:end-res,:);
                    trajectory = [initial_move_joint_trajectory; straight_joint_trajectory]; 
                    return
                end                
                
                % create joint trajectory (thetas)
                straight_joint_trajectory = zeros(num_trajectory_rows,5); 
                % loop through waypoints & perform IK                
                for  i = 1:size(straight_joint_trajectory,1)
                    goal_pos(1:3) = planning.straight_waypoint_data(i,:)';
                    goal_pos(4,1) = 1.68; % tune for pitch about y
                    goal_pos(5,1) = 0; % tune for yaw about x 
                    ik_thetas = planning.ik(initial_thetas, goal_pos);                      
                    straight_joint_trajectory(i,:) = ik_thetas;                    
                    % reset arguments for IK
                    initial_thetas = ik_thetas;                     
                end
                trajectory = [initial_move_joint_trajectory; straight_joint_trajectory];
                
                
            end
            
            if save_file
                planning.save_trajectory_file(trajectory,"my_straight_trajectory.csv");
            end
                                                         
        end
        
        %{
        function [trajectory] = create_straight(initial_thetas, goal_position, vars, resolution)
            % Creates trajectory to move the robot from current position 
            % to setup position where it can begin following a desired path
            % trajectory
            % inputs:
            %   thetas_pos = position that IK should start solving from
            if size(setup_position, 2) ~= 1
                error('Expecting a workspace position column vector.');
            end
            goal_pos = zeros(vars,1); % ee goal position                                    
            % IK for linear workspace trajectory into straight path            
            theta_configs = zeros(resolution,vars);            
            initial_move_joint_trajectory = zeros(resolution,vars);
            % create linear trajectory  
            for i = 1:size(initial_move_positions,2)-1
                initial_move_positions(:,i) = linspace(initial_pos(i), ...
                    setup_position(i,1), resolution);
            end
            % run IK
            for  i = 1:size(initial_move_positions,1)
                goal_pos(1:vars,:) = initial_move_positions(i,:)';            
                ik_thetas = planning.ik(initial_thetas, goal_pos); %% ee position from ik
                %end    
                initial_move_joint_trajectory(i,:) = ik_thetas;
                ee = planning.robot.ee(ik_thetas);        
                
                initial_move_ee_positions(i,1) = ee(1);
                initial_move_ee_positions(i,2) = ee(2);
                initial_move_ee_positions(i,3) = ee(3);
                initial_move_ee_positions(i,4) = ee(4);
                initial_move_ee_positions(i,5) = ee(5);
                
                % reset arguments for IK
                initial_thetas = ik_thetas;
                %initial_pos = ee; 
            end
            
        end
        %}
        
        
        
    end
end
