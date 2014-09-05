%robot = neato('nano');
%pause(2);
%robot.startLaser();
%pause(2);
%disp('Started Laser');
n = 1;
r = zeros(360,1);

kp1 = 0.3;          %Proportional term for linear velocity
kp2 = 1/45;         %Proportional term for angular velocity

fol = 0.50;      %following distance in m
vl = 0;             %left wheel velocity
vr = 0;             %right wheel velocity
left_array = zeros(500,1);      %array of left wheel velocities
right_array = zeros(500,1);     %array of right wheel velocities
dist_array = zeros(500,1);      %array of minimum distance obstacle
%figure(1);
x_array = zeros(500,1);         %array of x coordinate of min dist obstacle
y_array = zeros(500,1);         %array of y coordinate of min dist obstacle

while true
r = robot.laser.data.ranges;
[min_i, min_r] = minDistance(r);
[x y b] = irToXy(min_i,min_r);
clf;
scatter(0,0,'.');
hold on;
scatter(-100*y,100*x,'x');
if min_i~=0
    V = kp1*(min_r-fol);
    omega = kp2*b;
    [vl,vr] = vwtolr(V,omega);
end
x_array(n) = x;
y_array(n) = y;
robot.sendVelocity(vl,vr);
left_array(n) = vl;
right_array(n) = vr;
dist_array(n) = min_r;
pause(0.02);
n = n+1;
end
robot.stopLaser();
robot.close();