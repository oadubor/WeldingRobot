function [] = gravity_comp_testing()
    clear all;
    robot = Robot3D();        
    control = RobotControl3D(robot, 'Robot B');
    planning = RobotPlanning3D(robot);    
    control.fix_hebilookup();
    control.set_gains = true;
    control.gains();
    run_trajectory = true;    
    %control.gravity_compensation(robot.a_straight_initial_thetas);
    
    %control.homePositioning(robot_object.straight_initial_thetas, 100);
    if run_trajectory
        % Get initial thetas    
        robot_hardware = control.robot_connection();
        fbk = robot_hardware.getNextFeedback();
        initial_thetas = fbk.position';
        %initial_position = robot.ee(initial_thetas)            
        %setup_position = robot_object.ee(robot_object.a_straight_initial_thetas);    
        %first_waypoint_position = [0.2;0.6623;0.27]; % from first row of straight trajectory                
        % Create approach offset
        %y_offset_approach = -0.02; % for approach, makes robot insert into path slowly
        
        %{
        x_approach_offset = -0.02+0.02;
        y_approach_offset = -0.02+0.02; % for approach, makes robot insert into path slowly
        z_approach_offset = 0;
        setup_position = first_waypoint_position + [...
            x_approach_offset; y_approach_offset; z_approach_offset]; % modify approach
        %}
        setup_position = [0.2994; 0.8649; 1.0318; -0.1222; 0.4180];
              
        % IK functino arguments
        full_path = true;
        log_data = true;
        approach_resolution = 100;
        trajectory_resolution = 15;
        %expand_waypoints = true;
        frequency = 100;
        y_trajectory_offset = 0.02;  
        x_trajectory_offset = -0.02;
        z_trajectory_offset = -0.06;        
        
        % solve and follow path
        trajectory = planning.create_straight_trajectory(initial_thetas, ...
                    setup_position, 5, full_path, approach_resolution, ...
                    trajectory_resolution,x_trajectory_offset, ...
                    y_trajectory_offset, z_trajectory_offset, false);
        control.command_trajectory(robot_hardware, trajectory, frequency, log_data);
        fbk = robot_hardware.getNextFeedback();
        thetas = fbk.position';
        robot.ee(thetas);         
    else
        control.gravity_compensation(robot.a_straight_initial_thetas);
    end
end
% DEMO Parameters
%{
y_trajectory_offset = 0.02;  
x_trajectory_offset = -0.02;
z_trajectory_offset = -0.06;
approach_resolution = 100;
trajectory_resolution = 15;
setup_position = [0.2994; 0.8649; 1.0318; -0.1222; 0.4180];
robot.grav_masses = [0; 0.65; 0.2; 0.8; 0.02 + robot.ee_mass]; % joint masses
planning.yaw_orientation_ik_weight = 0.001;
planning.pitch_orientation_ik_weight = 0.05;
%}
