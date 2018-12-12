function [] = gravity_comp_testing()
    clear all;
    robot = Robot3D();        
    control = RobotControl3D(robot, 'Robot B');
    planning = RobotPlanning3D(robot);    
    control.fix_hebilookup();
    control.set_gains = true;
    control.gains();
    run_trajectory = true;    
    %control.gravity_compensation(robot_object.a_straight_initial_thetas);
    
    %control.homePositioning(robot_object.straight_initial_thetas, 100);
    if run_trajectory == true
        % Get initial thetas    
        robot_hardware = control.robot_connection();
        fbk = robot_hardware.getNextFeedback();
        initial_thetas = fbk.position';
        %initial_position = robot.ee(initial_thetas)    
        
        %setup_position = robot_object.ee(robot_object.a_straight_initial_thetas);    
        setup_position = [0.2;0.6623;0.27]; % from first row of straight trajectory
        %setup_position = [0.1800;0.5;0.27];
        setup_position(4,1) = 1.68; % tune for pitch about y
        setup_position(5,1) = 0; % tune for yaw about x        
        %setup_position
        %initial_thetas = [0.0776; 0.0109; 1.1410; 0.1628; 0.1879];
        resolution = 100;
        full_path = true;
        log_data = true;
        approach_resolution = 100;
        trajectory_resolution = 5;
        trajectory = planning.create_straight_trajectory(initial_thetas, ...
                    setup_position, 5, full_path, approach_resolution, trajectory_resolution, false);
        control.command_trajectory(robot_hardware, trajectory, 100, log_data);
        fbk = robot_hardware.getNextFeedback();
        thetas = fbk.position';
        robot.ee(thetas);       
        %while true
        %    robot_hardware.setCommandLifetime(0);
        %end
    else
        control.gravity_compensation(robot.a_straight_initial_thetas);
    end
end
