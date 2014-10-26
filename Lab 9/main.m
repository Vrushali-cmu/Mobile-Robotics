figure(1)
kp = robotKeypressDriver(gcf);
vGain = 1;
robot_name = 'nano';
robot = neato(robot_name);
pause(2);
tstart = tic;
%robotKeypressDriver.drive(robot,vGain);

while true
    robotKeypressDriver.drive(robot,vGain);
    if toc(tstart)>60
        disp('Time up');
        robot.sendVelocity(0,0);
        break;
    end
    
    
    pause(0.01);
end
