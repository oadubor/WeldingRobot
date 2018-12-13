classdef RobotControl3D 
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'public')                
        robot_joints
        robot_name
        robotA_name
        robotB_name
        robot_class       
        kp_gains
        ki_gains
        kd_gains
        frequency
        set_gains % boolean for setting gains
        robotA_gains_file
        robotB_gains_file
    end
    
    methods
        % Constructor 
        function control = RobotControl3D(robot_class,robot_name) 
            control.robot_name = robot_name;
            control.robotA_name = 'Robot A';
            control.robotB_name = 'Robot B';
            control.robot_joints = {'J1', 'J2', ...
                'J3', 'J4', 'J5'};
            control.robot_class = robot_class; 
            
            
            control.kp_gains = [4 15 3 10 2];
            
            
            %control.kp_gains = [4 15 3 4 2];
            control.ki_gains = [0.1 2 0.1 0.1 0.01];
            control.kd_gains = [0.005 0.1 0.1 0.1 0]; 
            control.frequency = 100; % Hz
            control.robotA_gains_file = 'robotA_gains.mat';
            control.robotB_gains_file = 'robotB_gains.mat';
            
            
            control.set_gains = false; % modify robot gains from file
        end
        
        function [] = fix_hebilookup(~)
            % Gets module structure for HebiLookup            
            HebiLookup.setLookupAddresses('*');
            HebiLookup.clearModuleList();
            HebiLookup.clearGroups();
            HebiLookup();
        end
        
        function [] = gains(control)
            % gets and set robot gains 
            robot = HebiLookup.newGroupFromNames(control.robot_name ...
                , control.robot_joints);  
            %gains_struct = robot.get('gains');
            % save('gains_file.mat','gains_struct'); % save gain struct in
            % file
            %display(gains_struct) % will show you other fields you can modify
            if control.robot_name == control.robotA_name
                file = control.robotA_gains_file;
            elseif control.robot_name == control.robotB_name
                file = control.robotB_gains_file;
            else
                disp('Robot name not recognized, NO gains will be set');
                return
            end
            gains_struct_wrapper = load(file); %contains gains_struct
            gains_struct = gains_struct_wrapper.gains_struct;
            display(gains_struct) % Display gains
            if control.set_gains
                gains_struct.positionKp = control.kp_gains;      
                gains_struct.positionKi = control.ki_gains;     
                gains_struct.positionKd = control.kd_gains;                
            end
            robot.set('gains', gains_struct); % note this is the set function, not send
        end   
        
        function [ torque ] = get_joint_torques(control,thetas,desiredForce,joint)
            % get_joint_torques
            %
            %   Calculates the joint torques required to result in a desired force
            %   vector (in world coordinates).            
            if size(desiredForce, 1) ~= 6 || size(desiredForce, 2) ~= 1
                error('Invalid desiredForce: Should be a column vector of 6 x 1, is %dx%d.', ...
                    size(desiredForce, 1), size(desiredForce, 2));
            end
            if size(thetas, 1) ~= control.robot_class.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', ...
                    size(thetas, 1), size(thetas, 2));
            end                  
            % thetas checked            
            J = control.robot_class.jacobians(thetas); % jacobian              
            J_T = transpose(J(:,:,joint));            
            torque = J_T*desiredForce;                        
        end
        
        function [robot] = robot_connection(control)
            robot = HebiLookup.newGroupFromNames(control.robot_name ...
                , control.robot_joints);                        
            warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
            input('Once ready, press "enter" to continue; press "ctrl-c" when finished...','s');
        end
        
        function [] = gravity_compensation(control, desired_position)
            % Used for mostly debugging and testing purposes
            % helper = whether to output torques for trajectories or run 
            %          independently
            % inputs
            %   desired_position = col vector theta configuration
            % gravity_compensation            
            %   This function makes the robot "weightless", extering joint torques that
            %   counteract the effect of gravity on the system.            
            % Robot Connection  
            robot = control.robot_connection(); % connect to robot hardware        
            % Define variables for gravity:
            gravity = [0;0;9.81;0;0;0]; % [m/s^2]
            % Setup reusable structures to reduce memory use in loop
            cmd = CommandStruct();
            tmpFbk = robot.getNextFeedback();                        
            % Loop, sending commands to the robot, until 'ctrl-c' is pressed.
            forces = zeros(6,1,control.robot_class.dof);
            torques = zeros(control.robot_class.dof,1,control.robot_class.dof);
            for i = 1:length(control.robot_class.grav_masses)                
                forces(:,1,i) = [...
                    0;0;control.robot_class.grav_masses(i);0;0;0].*gravity;
            end
            while true
                % Read Robot feedback
                fbk = robot.getNextFeedback(tmpFbk);
                thetas = fbk.position % get thetas   
                control.robot_class.ee(thetas'); % print ee position/orientation
                %return
                for i = 1 : control.robot_class.dof                
                    torques(:,1,i) = control.get_joint_torques(...
                        thetas',forces(:,1,i),i);
                end
                total_torque = sum(torques,3);                                              
                %cmd.torque = total_torque'; % convert to row vector                                                
                %cmd.position = control.robot_class.straight_initial_thetas';
                %cmd.position = desired_position';
                %cmd.position = [0 0.5 0 0 0];
                robot.set(cmd);
                % Wait a little bit; here we'll cap command rates to 100Hz.
                pause(0.1); % 100Hz frequency                
            end
        end
        
        function [torque] = trajectory_gravity_compensation(control, thetas)            
            % gravity_compensation            
            %   This function makes the robot "weightless", extering joint torques that
            %   counteract the effect of gravity on the system.
            % Function RETURNS ROW VECTOR
            % Define variables for gravity:
            gravity = [0;0;9.81;0;0;0]; % [m/s^2]                        
            % Loop, sending commands to the robot, until 'ctrl-c' is pressed.
            forces = zeros(length(gravity),1,control.robot_class.dof);
            torques = zeros(control.robot_class.dof,1,control.robot_class.dof);
            for i = 1:length(control.robot_class.grav_masses)                
                forces(:,1,i) = [...
                    0;0;control.robot_class.grav_masses(i);0;0;0].*gravity;
            end                                            
            for i = 1 : control.robot_class.dof                
                torques(:,1,i) = control.get_joint_torques(...
                    thetas,forces(:,1,i),i);
            end
            torque = (sum(torques,3))';                                                                                      
        end                
        
        function [trajectory] = homePositioning(control, homePosition, frequency)             
            robot = control.robot_connection();    
            %Get initial position
            fbk = robot.getNextFeedback();
            initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
            % Move shoulder to safe location
            homingTime = 2;
            firstSafeTrajectory = repmat(initial_thetas, 1, homingTime*frequency);
            firstSafeTrajectory(2,:) = linspace(initial_thetas(2), homePosition(2), homingTime*frequency);
            control.command_trajectory(robot, firstSafeTrajectory, frequency, false);

            % Move remaining joints to safe loacation
            % Get initial position
            fbk = robot.getNextFeedback();
            initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
            trajectory = zeros(size(firstSafeTrajectory));
            for i = 1:size(homePosition,1)
                trajectory(i,:) = linspace(initial_thetas(i), homePosition(i), homingTime*frequency);
            end
            control.command_trajectory(robot, trajectory, control.frequency, false);
        end
        
        function [] = command_trajectory(control, robot_hardware,...
                trajectory, frequency, log_data)
            % Setup reusable structures to reduce memory use in loop
            cmd = CommandStruct();
            % Log Robot Data
            if log_data 
                currentDir = fileparts(mfilename('fullpath'));
                robot_hardware.startLog('file', fullfile(currentDir, 'robot_data'));
            end
            % Compute the velocity numerically
            trajectory_vel = diff(trajectory, 1, 1);          
            % Command the trajectory
            for i = 1:(size(trajectory, 1) - 1)
                fbk = robot_hardware.getNextFeedback();  
                % Send command to the robot (the transposes on the trajectory
                % points turns column into row vector for commands).
                cmd.position = trajectory(i,:);
                cmd.torque = control.trajectory_gravity_compensation(fbk.position');
                %cmd.velocity = trajectory_vel(i,:) * frequency;
                robot_hardware.set(cmd);
                % Wait a little bit to send at ~100Hz.
                pause(1 / frequency);
            end
            % Send the last point, with a goal of zero velocity.
            cmd.position = trajectory(end,:);
            cmd.velocity = zeros(1, size(trajectory, 2));
            robot_hardware.set(cmd);
            % Data Logging Code Below
            if log_data % Stop logging, and save results               
                robot_hardware.stopLog();
                HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));                                                              
            end
        end
    end
end
