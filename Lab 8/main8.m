
global re;              %right encoder
global le;              %left encoder
global t_robot;               %timestamp
global vl_real;         %left wheel velocity
global vr_real;         %right wheel velocity


%use kv = 0.4, kw = 0.8, pause(0.005)
xf = [0.25 -0.5 -0.25];
yf = [0.25 -0.5 0.25];
thf = [0 -pi/2 pi/2];
fb = 1;

feedback_array = zeros(2,1000);
error_array = zeros(2,1000);
xfw = zeros(3,1);
yfw = zeros(3,1);
thfw = zeros(3,1);

xfw(1) = xf(1);
yfw(1) = yf(1);
thfw(1) = thf(1);
thfw(2) = thfw(1)+thf(1);
thfw(3) = thfw(2)+thf(2);
for i = 2:size(xf,2)
	th = thfw(i-1);
	pose = [cos(th) sin(th) xfw(i-1); -sin(th) cos(th) yfw(i-1)]*[xf(i); yf(i); 1];
	xfw(i) = pose(1);
	yfw(i) = pose(2);
end
	

Vmax = 0.05;
kv = 0.4*Vmax/0.25;
kw = 0.8*Vmax/0.25;
robot_name = 'nano';
robot = neato(robot_name);
pause(2);
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderListener);
curve = cell(3,1);
figure(1)
th_prev = thf(1);
for i = 1:3
    sgn = 1;
    curve{i} = cubicSpiral.planTrajectory(xf(i),yf(i),thf(i),sgn);
    x = curve{i}.poseArray(1,:);
    y = curve{i}.poseArray(2,:);
    if i==1
	x_prev = 0;
	y_prev = 0;
    else
	x_prev = xfw(i-1);
	y_prev = yfw(i-1);
    end
    world_poseArray = [cos(thfw(i)) -sin(thfw(i)) x_prev; sin(thfw(i)) cos(thfw(i)) y_prev]*[x;y;ones(1,size(x,2))];
    curve{i}.worldArray = world_poseArray;
    axis equal;
    axis([-0.5 0.5 -0.5 0.5]);
    plot(world_poseArray(1,:),world_poseArray(2,:));
    hold on
end

x_real = 0;
y_real = 0;
th_real = 0;
    
end_points = zeros(2,3); 

for i = 1:3
    curve{i}.planVelocities(Vmax);
    tstart = [];
    prev_t = [];
    dt = 0;

n = 0
    while true
        n = n+1;
        if isempty(tstart)
            tstart = tic;
        end
        t_now = toc(tstart);
        vl = curve{i}.getvlAtTime(t_now);
        vr = curve{i}.getvrAtTime(t_now);
        if t_now>(curve{i}.timeArray(end)+1)
            break;
            
        elseif t_now>(curve{i}.timeArray(end))
            vl = 0;
            vr = 0;
            
        end
        
        if fb~=0
            position_desired = curve{i}.getWorldAtTime(t_now);
            x_desired = position_desired(1);
            y_desired = position_desired(2);
            err_x_world = x_desired-x_real;
            err_y_world = y_desired-y_real;
            error_robot = [cos(th_real) sin(th_real); -sin(th_real) cos(th_real)]*[err_x_world;err_y_world];
            err_x_robot = error_robot(1);
            err_y_robot = error_robot(2);
            v_feedback = kv*err_x_robot;
            if err_x_robot>0.005
                w_feedback = kw*err_y_robot/err_x_robot;
            else
                w_feedback = 0;
            end
            feedback_array(:,i) = [v_feedback;w_feedback];
            error_array(:,i) = [err_x_robot; err_y_robot];
            [vl_feedback, vr_feedback] = vwtolr(v_feedback,w_feedback);
            vl = vl+vl_feedback;
            vr = vr+vr_feedback;
            
        end 
        if abs(vl)>0.3
            vl = sign(vl)*0.3;
        end
        if abs(vr)>0.3
            vr = sign(vr)*0.3;
        end
        robot.sendVelocity(vl,vr);
        [v,w] = lrtovw(vl,vr);
        th_real = th_real+w*dt*0.5;
        x_real = x_real+v*cos(th_real)*dt;
        y_real = y_real+v*sin(th_real)*dt;
        th_real = th_real+w*dt*0.5;
        scatter(x_real, y_real, '.', 'b');
        hold on;
        if isempty(prev_t)
            dt = toc(tstart)-t_now;
        else
            dt = t_now-prev_t;
        end
        prev_t = t_now;
        pause(0.005);
    end
    robot.sendVelocity(0,0);
    scatter(x_real,y_real,'x','r');
    end_points(:,i) = [x_real,y_real]; 
    hold on;
    pause(2);
end
        
        
        
        
        
        
        
        
    
    
    