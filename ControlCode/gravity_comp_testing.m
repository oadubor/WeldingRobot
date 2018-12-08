function [] = gravity_comp_testing()
    clear all;
    robot = Robot3D();        
    control = RobotControl3D(robot, 'Robot B');
    control.fix_hebilookup();
    control.gains();
    control.gravity_compensation();
end
