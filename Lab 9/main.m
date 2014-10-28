figure(1)
kp = robotKeypressDriver(gcf);
vGain = 0.5;
%robot_name = 'nano';
%robot = neato(robot_name);
%pause(2);
tstart = tic;
scale = 10;
%robotKeypressDriver.drive(robot,vGain);
lines_p1 = [0 0;1.3 0];
lines_p2 = [0 1.3;0 0];
%robot.startLaser();
%pause(5);
robotPose = pose(15*0.0254,9*0.0254,pi/2);
gain = 0.04;
errThresh = 0.005;
gradThresh = 0.0005;
maxIters = 200;
lml = lineMapLocalizer(lines_p1, lines_p2,gain,errThresh, gradThresh);
bodyPts = robotModel.bodyGraph();
laserpts = robot.laser.data.ranges;
[x,y,~] = irToXy(laserpts);

worldRobotPts = robotModel.robToWorld(robotPose)*bodyPts;
plot(worldRobotPts(1,:),worldRobotPts(2,:),'g');
hold on;
plot([0 0 1.3], [1.3 0 0],'b');

%scatter(worldLidarPts(1,:),worldLidarPts(2,:),'.','r');
    %hold on;
axis([-1 1.5 -1 1.5]);
axis equal;
pause(0.01);
x = x(1:scale:end);
y = y(1:scale:end);
points = [x; y; ones(1, numel(x))];
worldLidarPts = robotModel.senToWorld(robotPose)*points;
[success, outPose] = refinePose(lml,robotPose,points, maxIters);
if success~=0
    
    robotPose = pose(outPose.x,outPose.y,outPose.th);
    scatter(worldLidarPts(1,:),worldLidarPts(2,:),'.','r');
    hold on;
    %worldRobotPts = robotModel.robToWorld(robotPose)*bodyPts;
    worldRobotPts = robotPose.bToA()*bodyPts;
    plot(worldRobotPts(1,:),worldRobotPts(2,:),'g');
    hold on;
    plot([0 0 1.3], [1.3 0 0],'b');
    axis([-1 1.5 -1 1.5]);
    axis equal
    pause(0.01);
    
end
robot.sendVelocity(0,0);
while true
    
    
    robotKeypressDriver.drive(robot,vGain);
    if toc(tstart)>60
        disp('Time up');
        robot.sendVelocity(0,0);
        break;
    end
    bodyPts = robotModel.bodyGraph();
    laserpts = robot.laser.data.ranges;
    [x,y,~] = irToXy(laserpts);
    x = x(1:scale:end);
    y = y(1:scale:end);
    
    points = [x; y; ones(1, numel(x))];
    worldLidarPts = robotModel.senToWorld(robotPose)*points;
    [success, outPose] = refinePose(lml,robotPose,points, maxIters);
    if success==1
        robotPose = pose(outPose.x,outPose.y,outPose.th);
    end
    clf;
    scatter(worldLidarPts(1,:),worldLidarPts(2,:),'.','r');
    hold on;
    worldRobotPts = robotPose.bToA()*bodyPts;
    %worldRobotPts = robotModel.robToWorld(robotPose)*bodyPts;
    plot(worldRobotPts(1,:),worldRobotPts(2,:),'g');
    hold on;
    plot([0 0 1.3], [1.3 0 0],'b');
    axis equal
    axis([-0.1 1.5 -0.1 1.5]);
    pause(0.01);
end
robot.stopLaser;
robot.sendVelocity(0,0);

