classdef Robot3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        dh_parameters
        LB
        UB
        grav_masses % joint masses for gravity compensation
        ee_mass
        rest_initial_thetas % joint config w/no gravity compensation
        straight_initial_thetas
        sine_initial_thetas
        a_straight_initial_thetas
    end
    
    methods
        % Constructor: Makes a b rand new robot with the specified parameters.
        function robot = Robot3D() % [a, alpha, d, theta]            
            % Link Lengths
            a = 116.23/1000;
            b = 91.05/1000;
            c = 327.76/1000;
            d = 91.05/1000;
            e = 94.05/1000;
            f = 279.5/1000;
            g = 57.4/1000;
            h = 266.7/1000;            
            % Store dh parameters
            robot.dh_parameters = [0, pi/2, a, 0;
                                   c, pi, b, 0;
                                   0, pi/2, d, pi/2;
                                   f, pi/2, e, pi/2;
                                   h, pi/2, g, 0];                             
            robot.LB = [-pi/6; deg2rad(-15); -2*pi; -pi; -Inf]; % lower bound for IK
            robot.UB = [deg2rad(200); deg2rad(200); 2*pi; pi; Inf]; % upper bound for IK            
            %robot.LB = [-pi; -pi; -pi; -pi; -Inf]; % lower bound for IK
            %robot.UB = [pi; pi; pi; pi; Inf]; % upper bound for IK            
            robot.dof = size(robot.dh_parameters, 1);
            robot.ee_mass = 0; % additional mass to add to ee during demo
            robot.grav_masses = [0; 1; 0; 0.8; 0.02 + robot.ee_mass]; % joint masses
            % saved joint configurations FOR ROBOT B
            robot.rest_initial_thetas = [-0.0489; 1.4914; 0.0701; 0.0363; -1.5853];            
            robot.straight_initial_thetas =  [0.2624; 0.91; 1.1358; -0.065; 0.4091];
            robot.sine_initial_thetas = [0.3485; 0.8325; 1.0702; -0.114; 0.5085];
            % saved joint configurations FOR ROBOT A
            robot.a_straight_initial_thetas = [0.3787; 0.9484; 1.0940; -0.0182; 0.4145];
        end
        
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
                        
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).
 
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 4x4 matrix frames(:,:,end).

            %% FILL IN 4x4 HOMOGENEOUS TRANSFORM FOR n FRAMES            
            % H^i-1_i = [Rot(z),theta]*[Trans(z),d]*[Trans(x),a]*[Rot(x),alpha]
            DH = robot.dh_parameters(1,:); % extract D-H paramaters of frame 1
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % homogeneous  rotation about z 
            H_rot_z = eye(4,4);
            % calculate angles            
            c_theta = cos(DH(4)+thetas(1));
            s_theta = sin(DH(4)+thetas(1));
            % save angles to array
            rotations = num2cell([c_theta,-1*s_theta,s_theta,c_theta]);
            % distribute angles to respective indices
            [H_rot_z(1, 1), H_rot_z(1, 2), H_rot_z(2, 1), ...
                H_rot_z(2, 2)] = deal(rotations{:});
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % homogeneous translation along z 
            H_trans_z = eye(4,4); 
            H_trans_z(3,4) = DH(3);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % homogeneous translation along x 
            H_trans_x = eye(4,4); 
            H_trans_x(1,4) = DH(1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % homogeneous rotation about x 
            H_rot_x = eye(4,4);
            % calculate angles
            c_theta = cos(DH(2));
            s_theta = sin(DH(2));
            % save angles to array
            rotations = num2cell([c_theta,-1*s_theta,s_theta,c_theta]);
            % distribute angles to respective indices
            [H_rot_x(2, 2), H_rot_x(2, 3), H_rot_x(3, 2), ...
                H_rot_x(3, 3)] = deal(rotations{:});
            frames(:,:,1) = ((H_rot_z*H_trans_z)*H_trans_x)*H_rot_x;            
            for i = 2:n % loop through joints
                DH = robot.dh_parameters(i,:); % extract D-H paramaters of frame
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % homogeneous  rotation about z 
                H_rot_z = eye(4,4);
                % calculate angles
                c_theta = cos(DH(4)+thetas(i));
                s_theta = sin(DH(4)+thetas(i));
                % save angles to array
                rotations = num2cell([c_theta,-1*s_theta,s_theta,c_theta]);
                % distribute angles to respective indices
                [H_rot_z(1, 1), H_rot_z(1, 2), H_rot_z(2, 1), ...
                    H_rot_z(2, 2)] = deal(rotations{:});
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % homogeneous translation along z 
                H_trans_z = eye(4,4); 
                H_trans_z(3,4) = DH(3);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % homogeneous translation along x 
                H_trans_x = eye(4,4); 
                H_trans_x(1,4) = DH(1);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % homogeneous rotation about x 
                H_rot_x = eye(4,4);
                % calculate angles
                c_theta = cos(DH(2));
                s_theta = sin(DH(2));
                % save angles to array
                rotations = num2cell([c_theta,-1*s_theta,s_theta,c_theta]);
                % distribute angles to respective indices
                [H_rot_x(2, 2), H_rot_x(2, 3), H_rot_x(3, 2), ...
                    H_rot_x(3, 3)] = deal(rotations{:});
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Multiply homogeneous transforms
                % store transform from H_i-1_i in frames
                frames(:, :, i) = (((frames(:, :, i-1)*H_rot_z)*H_trans_z)*H_trans_x)*H_rot_x;
            end
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; z; yaw; pitch; roll] of ee
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);            
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);  
            % Gather relevant rotation matrix values
            r_11 = H_0_ee(1,1);            
            r_21 = H_0_ee(2,1);            
            r_31 = H_0_ee(3,1);     
            r_32 = H_0_ee(3,2);
            r_33 = H_0_ee(3,3);
            % Calculate roll, pitch, yaw
            yaw = atan2(r_21,r_11); % psi
            pitch = atan2(-r_31,sqrt(r_32^2+r_33^2)); % theta
            roll = atan2(r_32,r_33); % phi
            % Pack them up nicely.
            ee = [x; y; z; yaw; pitch; roll];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        function jacobians = jacobians(robot, thetas)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            % Allocate jacobian for base frame to end effector frame 
            %jacobians = zeros(6,robot.dof); 
            jacobians = zeros(6,robot.dof,5);             
    % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Create column vectors
            o_vector = zeros(3,6);
            z_vector = zeros(3,6);    
            % Get FK map
            base_frames = robot.fk(thetas);            
            %z_vector(:,1) = [0;0;1];           
            % Populate column vectors with FK map             
            for i = 1 : robot.dof
                o_vector(:,i) = base_frames(1:3,4,i);
                z_vector(:,i) = base_frames(1:3,3,i);                
            end                                                  
            
            for frame = 1 : length(base_frames) % loop through frames of robot
                %jacobians(1:3,1,frame) = cross([0;0;1],(o_vector(:,frame)-[0;0;0]));
                %jacobians(4:6,1,frame) = [0;0;1];       
                o_n = o_vector(:,frame);
                for joint = 1 : frame  % loop up to frame in outer for loop                      
                    if joint == 1
                        z_i_1 = [0;0;1];
                        o_i_1 = [0;0;0];
                    else
                        z_i_1 = z_vector(:,joint-1);                    
                        o_i_1 = o_vector(:,joint-1);
                    end
                    jacobians(1:3,joint,frame) = cross(z_i_1, ...
                        (o_n-o_i_1));                
                    jacobians(4:6,joint,frame) = z_i_1;                
                end
            end
            
        
% --------------- END STUDENT SECTION ------------------------------------
        end
        
        % Shorthand for returning the end effector jacobian. 
        function ee_j = ee_jacobian(robot, thetas)
            J = robot.jacobians(thetas);
            ee_j = J(:,robot.dof);
        end
                
        % Returns forces applied at end effector from joint torques
        function ee_forces = ee_forces(robot, thetas, torques)
            
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', ...
                   size(thetas, 1), size(thetas, 2));
            end
            if size(torques, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid torques: Should be a column vector matching robot DOF count, is %dx%d.', ...
                   size(torques, 1), size(torques, 2));
            end
            
            % Calculate jacobian
            J = robot.jacobians(thetas);
            % Calculate end effector forces 
            ee_forces = -transpose(J)\torques;
        end
        
        %{
        function c = cost(robot, thetas, goal_position) % IK Cost Function                 
                ee = robot.ee(thetas);
                multiplier = 0.9;
                ee_focus = [ee(1);ee(2);ee(3);multiplier*ee(4);multiplier*ee(5)];
                %goal_position
                c = sum((ee_focus-goal_position(1:5)).^2); % ignores orientation
        end
        
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', ...
                    size(initial_thetas, 1), size(initial_thetas, 2));
            end

            if (size(goal_position, 1) ~= 6 && size(goal_position, 2) ~= 1)
                error('Invalid goal_position: Should be a 6 length column vector, is %dx%d.', ...
                    size(goal_position, 1), size(goal_position, 2));
            end
            
            thetas = fmincon(@(thetas) robot.cost(thetas,goal_position),...
                initial_thetas,[],[],[],[],robot.LB,robot.UB); % found theta values
        
% --------------- END STUDENT SECTION ------------------------------------
        end
        
        % Shorthand for returning the end effector position and orientation of IK solution 
        function ik = ik(robot, initial_thetas, goal_position)
            ik = robot.inverse_kinematics(initial_thetas, goal_position);
        end
        %}
        
    end
end
