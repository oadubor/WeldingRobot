classdef RobotControl3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        %dof
        %dh_parameters        
        robot_joints
        robot_name
        robot_class
        %hebi_robot
       % HebiLookup
        %link_lengths
        %link_masses
        %joint_masses
        %end_effector_mass
    end
    
    methods
        % Constructor 
        function control = RobotControl3D(robot_class,robot_name) 
            control.robot_name = robot_name;
            control.robot_joints = {'J1', 'J2', ...
                'J3', 'J4', 'J5'};
            control.robot_class = robot_class;
            %control.hebi_robot = control.robot_connection(); % connect to robot
            %control.HebiLookup = HebiLookup;
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
            gains_struct_wrapper = load('gains_file.mat'); %contains gains_struct
            gains_struct = gains_struct_wrapper.gains_struct;
            %display(gains_struct) % Display gains
            %gains_struct.positionKp = [5 3 3 3 3];         
            %gains_struct.positionKi = [1 0.001 0 0.01 0.01];     
            %gains_struct.positionKd = [0.1 0.08 0.001 0.001 0.01];
            robot.set('gains', gains_struct); % note this is the set function, not send
        end            
        function [ torque ] = get_joint_torques(control,thetas,desiredForce,joint)
            % get_joint_torques
            %
            %   Calculates the joint torques required to result in a desired force
            %   vector (in world coordinates).
            % Get torques for motors 2,3,4
            if size(desiredForce, 1) ~= 6 || size(desiredForce, 2) ~= 1
                error('Invalid desiredForce: Should be a column vector matching robot DOF count, is %dx%d.', ...
                    size(desiredForce, 1), size(desiredForce, 2));
            end
            if size(thetas, 1) ~= control.robot_class.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', ...
                    size(thetas, 1), size(thetas, 2));
            end                  
            % thetas checked            
            J = control.robot_class.jacobians(thetas); % jacobian  
            %torque(2) = J(1:3,)
            %transpose(J(1:3,:))
            %desiredForce  
            J_T = transpose(J(:,:,joint));
            %if J_T(3) ~= 0
            torque = J_T*desiredForce;
            %else
            %    torque = 0;
            %end
            
        end
        
        function [robot] = robot_connection(control)
            robot = HebiLookup.newGroupFromNames(control.robot_name ...
                , control.robot_joints);                        
            warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
            input('Once ready, press "enter" to continue; press "ctrl-c" when finished...','s');
        end
        
        function [] = gravity_compensation(control)
            % gravity_compensation            
            %   This function makes the robot "weightless", extering joint torques that
            %   counteract the effect of gravity on the system.            
            % Robot Connection  
            robot = control.robot_connection();
            % Define variables for gravity:
            gravity = [0;0;9.81;0;0;0]; % [m/s^2]
            % Setup reusable structures to reduce memory use in loop
            cmd = CommandStruct();
            tmpFbk = robot.getNextFeedback();                        
            % Loop, sending commands to the robot, until 'ctrl-c' is pressed.
            while true
                % Read Robot feedback
                fbk = robot.getNextFeedback(tmpFbk);
                thetas = fbk.position; % get thetas
                %control.robot_class.forward_kinematics(thetas');
                %m_2 = 1.83;
                m_2 = 0.67;
                m_3 = 0;
                m_4 = 0.7;
                m_5 = 0.02;
                force2 = [0;0;m_2;0;0;0].*gravity;
                force3 = [0;0;m_3;0;0;0].*gravity;
                force4 = [0;0;m_4;0;0;0].*gravity; 
                force5 = [0;0;m_5;0;0;0].*gravity; 
                torque2 = control.get_joint_torques(thetas',force2,2);              
                torque3 = control.get_joint_torques(thetas',force3,3); 
                torque4 = control.get_joint_torques(thetas',force4,4); 
                torque5 = control.get_joint_torques(thetas',force5,5); 
                %cmd.torque = [0,torque2,torque3,torque4,0]; % Note: should be a 1x2 (row) vector.
                torque_total = (torque2+torque4)';
                %torque_total(3) = -0;
                %cmd.torque = [0,0,0,0,0];
                cmd.torque = torque_total; % Note: should be a 1x2 (row) vector.                
                % Send command to robot; limit velocity to damp out fast motions.
                %cmd.velocity = [0,0,0,0,0]; 
                cmd.position = [0,1,0,0,0];
                robot.set(cmd);
                % Wait a little bit; here we'll cap command rates to 100Hz.
                pause(0.1); % 100Hz frequency 
            end
        end
        function follow_trajectory(control, trajectory)
            % Use FK to follow trajectory in thetas
            %robot = control.robot_connection(); % connect to robot
            % Setup reusable structures to reduce memory use in loop
            cmd = CommandStruct();
            tmpFbk = robot.getNextFeedback();  
            for i = 1:length(trajectory)
                %fbk = robot.getNextFeedback(tmpFbk);
                %thetas = control.robot_class.forward_kinematics(trajectory(i,:)')
                cmd.velocity = [0,0,0,0,0];
                cmd.torque = [0,0,0,0,0];                
                %trajectory(i,:)
                %cmd.position = trajectory(i,:);
                robot.set(cmd);
                pause(0.1);
                %cmd.position = thetas'                                
            end
            
        end
    end
end