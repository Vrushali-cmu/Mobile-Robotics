global re;      %right encoder value
global le;      %left encoder value
global vl;      %left wheel velocity (actual)
global vr;      %right wheel velocity (actual)

%set up a listener. Listener automatically updates actual left and right
%velocity in the global
lh = event.listener(robot.encoders,'OnMessageReceived',@neatoEncoderEventListener);
pause(2);

%goalState = 100;
%actualState = 0;
%error = goalState-actualState;
%errorIntegral = 0;
%errorIntegralMax = 0.3;

%%kp = 0.002;
kd = 0.0002;
ki = 0.0001;
tf = 6.0;
tstart = tic;
t_prev = 0;

rstart = re;
lstart = le;

figure(1);clf;
ylabel('millimeters');
xlabel('seconds');
legend_str = sprintf('kp = %d\nkd = %d\nki = %d',kp,kd,ki);

while true
    t_now = toc(tstart);                            %computer's clock
    lasterror = error;                              
    actualState = ((le-lstart));
    error = goalState-actualState;
    dt = t_now - t_prev;
    errorDerivative = (error-lasterror)/dt;
    errorIntegral = errorIntegral + (error*dt);
    
    sign = (errorIntegral>=0) + (errorIntegral<0)*(-1);
    if abs(errorIntegral)>errorIntegralMax
        errorIntegral = errorIntegralMax*sign;
    end
    
    V = kp*error + kd*errorDerivative + ki*errorIntegral;

    sign_V = (V>=0) + (V<0)*(-1);
    if (abs(V)>0.3)
        V = 0.3*sign_V;
    end
    
    [vl_comp,vr_comp] = vwtolr(V,0);
    robot.sendVelocity(vl_comp,vr_comp);
    
    figure(1);
    scatter(t_now,error);
    legend(legend_str);
    hold on;
    
    if t_now>6.0
        robot.sendVelocity(0,0);
        break;
    end
    
    %if error<0.1
    %    robot.sendVelocity(0,0);
    %    break;
    %end
    
    t_prev=t_now;
    
    pause(0.01);
end

robot.sendVelocity(0,0);
delete(lh);             %delete the event listener
robot.close();