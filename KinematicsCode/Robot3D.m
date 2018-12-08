classdef Robot3D
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        dh_parameters
        LB
        UB
        %link_lengths
        %link_masses
        %joint_masses
        %end_effector_mass
    end
    
    methods
        % Constructor: Makes a b rand new robot with the specified parameters.
        function robot = Robot3D() % [a, alpha, d, theta]
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %{
            if size(link_lengths, 2) ~= 1
               error('Invalid link_lengths: Should be a column vector, is %dx%d.', size(link_lengths, 1), size(link_lengths, 2));
            end
            
            if size(link_masses, 2) ~= 1
               error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
               error('Invalid joint_masses: Should be a column vector.');
            end
            
            if ~isnumeric(end_effector_mass)
               error('Invalid end_effector_mass: Should be a number.'); 
            end
            %}            
            %{
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of link lengths.');
            end
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
            end
            
            robot.link_lengths = link_lengths;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
            robot.end_effector_mass = end_effector_mass;  
            %}
            % Link Lengths
            a = 116.23/1000;
            b = 91.05/1000;
            c = 327.76/1000;
            d = 91.05/1000;
            e = 94.05/1000;
            f = 279.5/1000;
            g = 57.4/1000;
            h = 266.7/1000;
            %initial_thetas = [0;0;0;0;0];
            % Store dh parameters
            robot.dh_parameters = [0, pi/2, a, 0;
                                c, pi, b, 0;
                                0, pi/2, d, pi/2;
                                f, pi/2, e, pi/2;
                                h, pi/2, g, 0];                             
            robot.LB = [-pi/6; deg2rad(-15); -2*pi; -pi; -pi/6]; % lower bound for IK
            robot.UB = [deg2rad(200); deg2rad(200); 2*pi; pi; pi/6]; % upper bound for IK            
            robot.dof = size(robot.dh_parameters, 1);
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
            %c_theta = cos(thetas(1));
            %s_theta = sin(thetas(1));
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
            % loop through columns of jacobian for end effector frame
            %for i = 1 : robot.do
            %{
            jacobians(1:3,1) = cross([0;0;1],(o_vector(:,6)-[0;0;0]));
            jacobians(4:6,1) = [0;0;1];
            for joint = 2 : robot.dof
                jacobians(1:3,joint) = cross(z_vector(:,joint-1), ...
                    (o_vector(:,6)-o_vector(:,joint-1)));                
                jacobians(4:6,joint) = z_vector(:,joint-1);                
            end
            %}       
            
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
        
        % Shorthand for returning the end effector position and orientation. 
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
        
        function c = cost(robot, thetas, goal_position) % IK Cost Function 
                %c = -1*transpose(robot.ee_jacobian(thetas))*(robot.ee(thetas)-goal_position);
                ee = robot.ee(thetas);
                c = sum((ee(1:3)-goal_position(1:3)).^2); % ignores orientation
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
        
    end
end
