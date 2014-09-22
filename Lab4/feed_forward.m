figure(1);clf;
ylabel('millimeters');
xlabel('seconds');
figure(2);clf;
ylabel('millimeters');
xlabel('seconds');
sref=0;
lstart=le;
t_delay=0

global tf;
global re;      %right encoder value
global le;      %left encoder value
global vl;      %left wheel velocity (actual)
global vr;      %right wheel velocity (actual)

%set up a listener. Listener automatically updates actual left and right
%velocity in the global
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);
pause(2);

tstart=tic;
t_prev=0;
while true
    t_now=toc(tstart);
    uref = trapezoidalVelocityProfile((t_now-t_delay), 0.3*0.25, 0.25, 1.0, 1.0);
    robot.sendVelocity(uref,uref);
    dt=t_now-t_prev;
    sref=sref+uref*dt;
    
    figure(1);
    scatter(t_now,sref);
    scatter(t_now,(le-lstart)/1000,'+')
    ylabel('meters');
    xlabel('seconds');
    legendstr = sprintf('t delay =  %d',t_delay);
    legend(legendstr);
    hold on;
    
    figure(2);
    scatter(t_now,uref);
    ylabel('meters/sec');
    xlabel('seconds');
    hold on;
    
    if t_now>tf
        robot.sendVelocity(0,0);
        break;
    end
    t_prev=t_now;
    pause(0.01);
end
robot.sendVelocity(0,0);
delete(lh);
robot.close();