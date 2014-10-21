global re;              %right encoder
global le;              %left encoder
global t_robot;               %timestamp
global vl_real;         %left wheel velocity
global vr_real;         %right wheel velocity

fb = 0;

Vmax = 0.25;
kv = 0.4*Vmax/0.25;
kw = 0.8*Vmax/0.25;
%{
robot_name = 'centi';
robot = neato(robot_name);
pause(2);
robot.startLaser();
pause(2);
%}
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderListener);
%start laser


for i = 1:1
    figure(i);
    range_data = robot.laser.data.ranges;
    pose = GetObstaclePose(range_data);
    xf = pose(1);
    yf = pose(2);
    thf = pose(3);
    if sqrt(xf^2+yf^2)<0.08
        disp('Object close enough');
        break;
    end
    curve = cubicSpiral.planTrajectory(xf,yf,thf,1);
    curve.planVelocities(Vmax);
    
    
    axis equal;
    axis([-1 1 -1 1]);
    plot(curve);
    hold on
    
    %plot([xf+0.1 xf-0.1],[yf-0.1/sl yf+0.1/sl]); 
    %hold on;
    tstart = [];
    prev_t = [];
    dt = 0;
    n = 0;
    x_real = 0;
    y_real = 0;
    th_real = 0;
    

while true
    if isempty(tstart);
        tstart = tic;
    end
    t_now = toc(tstart);
    vl = curve.getvlAtTime(t_now);
    vr = curve.getvrAtTime(t_now);
    if t_now>(curve.timeArray(end)+1)
        robot.sendVelocity(0,0);
        break;
    elseif t_now>curve.timeArray(end)
        vl = 0;
        vr = 0;
    end
    %{
    if fb~=0
        %position_desired = curve.getWorldAtTime(t_now);
        position_desired = curve.getPoseAtTime(t_now);
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
        %feedback_array(:,i) = [v_feedback;w_feedback];
        %error_array(:,i) = [err_x_robot; err_y_robot];
        [vl_feedback, vr_feedback] = vwtolr(v_feedback,w_feedback);
        vl = vl+vl_feedback;
        vr = vr+vr_feedback;  
    end 
    %}
    if abs(vl)>0.3
        vl = sign(vl)*0.3;
    end
    
    if abs(vr)>0.3
        vr = sign(vr)*0.3;
    end
    robot.sendVelocity(vl,vr);
    [v,w] = lrtovw(vl_real,vr_real);
    th_real = th_real+w*dt*0.5;
    x_real = x_real+v*cos(th_real)*dt;
    y_real = y_real+v*sin(th_real)*dt;
    th_real = th_real+w*dt*0.5;
    scatter(x_real, y_real, '.', 'b');
    axis([-1 1 -1 1]);
    axis equal;
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
pause(1);

end

        
        
        
    