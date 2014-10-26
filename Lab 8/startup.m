clear all
close all
robot_name = 'nano';
robot = neato(robot_name);
pause(2);
robot.startLaser();
pause(5);