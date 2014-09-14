global re;      %right encoder value
global le;      %left encoder value
global vl;      %left wheel velocity (actual)
global vr;      %right wheel velocity (actual)
x = 0;
y = 0;
th = 0;
x_arr = [];     %array of x velocities
y_arr = [];     %array of y velocities
th_arr = [];    %array of thetas
%v_actual = [];  %array of actual velocities

%set up a listener. Listener automatically updates actual left and right
%velocity in the global
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);
pause(2);

ks = 0.5;       
kv = 0.4;

%{
while true
    robot.sendVelocity(-0.15,0.15);
    if toc(tstart)>10
        robot.sendVelocity(0,0);
        break;
    end
    v_actual = [v_actual;0.5*(vl+vr)];
end
%}

tstart = tic;
t_prev = 0;
figure(1);
axis manual;
axis([-350 350 -350 350]);
xlabel('millimeters');
ylabel('millimeters');
while true
    t_comp = toc(tstart);       %computer's clock
    vr_com= (0.3*kv+0.14125*(kv/ks)*sin(t_comp*kv/(2*ks)));     %commanded left velocity      
    vl_com = (0.3*kv-0.14125*(kv/ks)*sin(t_comp*kv/(2*ks)));    %commanded right velocity
    robot.sendVelocity(vl_com,vr_com);                          %send commanded velocities
    [v,w] = lrtovw(vl,vr);          %find v and w from actual velocities
    dt = t_comp-t_prev;             %dt from last loop
    th = th+0.5*w*dt;               %update theta
    x = x + v*cos(th)*dt;           %update x
    y = y + v*sin(th)*dt;           %update y
    th = th + 0.5*w*dt;             %update theta again
    
    x_arr = [x_arr;x];              %append new value of x to the array
    y_arr = [y_arr;y];              %append new value of y to the array
    th_arr = [th_arr;th];           %append new value of theta to the array
    t_prev = t_comp;                %assign value of time to t_prev
    
    %for plotting the graph in real time
    axis manual;
    axis([-350 350 -350 350]);
    scatter(-y,x,'.','b');
    hold on;
    
    %send zero velocity and get out of the loop when time is over 
    if toc(tstart)>=12.565*ks/kv
        robot.sendVelocity(0,0);
        break;
    end
    
    pause(0.001);
end
%scatter(x_arr,y_arr,'.');
robot.sendVelocity(0,0);
delete(lh);             %delete the event listener
%plot(v_actual);