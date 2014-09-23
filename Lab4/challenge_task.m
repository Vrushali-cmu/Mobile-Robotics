figure(1);clf;
ylabel('millimeters');
xlabel('seconds');
figure(2);clf;
ylabel('millimeters');
xlabel('seconds');
sref=0;
t_delay=0.0
feedback_switch=1;

global tf;
global re;      %right encoder value
global le;      %left encoder value
global vl;      %left wheel velocity (actual)
global vr;      %right wheel velocity (actual)

%set up a listener. Listener automatically updates actual left and right
%velocity in the global
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);
pause(2);


lstart=le;
goalState = 1000;
error=0;
actualState = 0;
errorIntegral = 0;
errorIntegralMax = 0.3;

kp = 0.004;
kd = 0.0002;
ki = 0.00015;

tstart=tic;
t_prev=0;
while true
    t_now=toc(tstart);
    %Feed-forward
    uref = trapezoidalVelocityProfile((t_now-t_delay), 0.3*0.25, 0.25, 1.0, 1.0);
    dt = t_now - t_prev;
    sref=sref+uref*dt;
    
    
    %Feed-back
    lasterror = error;                              
    actualState = ((le-lstart));
    error = sref*1000-actualState;
    
    if feedback_switch==1    
        errorDerivative = (error-lasterror)/dt;
        errorIntegral = errorIntegral + (error*dt);
    
        sign = (errorIntegral>=0) + (errorIntegral<0)*(-1);
        if abs(errorIntegral)>errorIntegralMax
            errorIntegral = errorIntegralMax*sign;
        end
    
        V = kp*error + kd*errorDerivative + ki*errorIntegral;
        V = uref+V;
    else
        V=uref;
    end
    
    sign_V = (V>=0) + (V<0)*(-1);
    if (abs(V)>0.3)
        V = 0.3*sign_V;
    end
    
    robot.sendVelocity(V,V);
    
    figure(1);
    scatter(t_now,sref);
    scatter(t_now,(le-lstart)/1000,'+')
    ylabel('meters');
    xlabel('seconds');
    legendstr = sprintf('t delay =  %d',t_delay);
    legend(legendstr);
    hold on;
    
    figure(2);
    scatter(t_now,error);
    ylabel('millimeters');
    xlabel('seconds');
    hold on;
    
    if t_now>(tf+1)
        robot.sendVelocity(0,0);
        break;
    end
    t_prev=t_now;
    %pause(0.001);
end
robot.sendVelocity(0,0);
delete(lh);
robot.close();