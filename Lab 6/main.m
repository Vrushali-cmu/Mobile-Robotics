
global re;              %right encoder
global le;              %left encoder
global t_robot;               %timestamp
global vl_real;         %left wheel velocity
global vr_real;         %right wheel velocity

xf = [0.25 -0.5 -0.25];
yf = [0.25 -0.5 0.25];
thf = [0 -pi/2 pi/2];

Vmax = 0.25;

robot_name = 'nano';
robot = neato(robot_name);
pause(2);
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderListener);
curve = cell(3,1);
for i = 1:3
    sgn = 1;
    curve{i} = cubicSpiral.planTrajectory(xf(i),yf(i),thf(i),sgn);
    figure(i);
    axis equal;
    axis([-1 1 -1 1]);
    plot(curve{i});
    hold on
end



for i = 1:3
    curve{i}.planVelocities(Vmax);
    tstart = [];
    prev_t = [];
    dt = 0;
    figure(i);
    x_real = 0;
    y_real = 0;
    th_real = 0;
    while true
        if isempty(tstart)
            tstart = tic;
        end
        t_now = toc(tstart);
        if t_now>curve{i}.timeArray(end)
            break;
        end
        vl = curve{i}.getvlAtTime(t_now);
        vr = curve{i}.getvrAtTime(t_now);
        robot.sendVelocity(vl,vr);
        [v,w] = lrtovw(vl,vr);
        th_real = th_real+w*dt*0.5;
        x_real = x_real+v*cos(th_real)*dt;
        y_real = y_real+v*sin(th_real)*dt;
        th_real = th_real+w*dt*0.5;
        scatter(x_real, y_real, '.','b');
        hold on;
        if isempty(prev_t)
            dt = toc(tstart)-t_now;
        else
            dt = t_now-prev_t;
        end
        prev_t = t_now;
        pause(0.01);
    end
    robot.sendVelocity(0,0);
    pause(1);
end
        
        
        
        
        
        
        
        
    
    
    