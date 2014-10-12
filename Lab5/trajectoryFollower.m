classdef trajectoryFollower < handle
    properties
      rob  
      vmax
      amax
      x_arr
      y_arr
      f
      err_x
      err_y
      v_forward
      v_back
      w_forward
      w_back
    end
    methods
        function obj = trajectoryFollower(robot_name,feedback)
            obj.rob = neato(robot_name);
            pause(2);
            obj.vmax = 0.15;
            obj.amax = 3*obj.vmax;
            obj.x_arr = zeros(1000,1);
            obj.y_arr = zeros(1000,1);
            obj.err_x = zeros(1000,1);
            obj.err_y = zeros(1000,1);
            obj.f = feedback;
            obj.v_forward = zeros(1000,1);
            obj.v_back = zeros(1000,1);
            obj.w_forward = zeros(1000,1);
            obj.w_back = zeros(1000,1);
        end
        
        function setXY(obj,x,y,i)
            obj.x_arr(i) = x;
            obj.y_arr(i) = y;
        end
        
        function set_error(obj,errx,erry,i)
            obj.err_x(i) = errx;
            obj.err_y(i) = erry;
        end
        
        function set_vw(obj,vfor,vback,wfor,wback,i)
            obj.v_forward(i) = vfor;
            obj.v_back(i) = vback;
            obj.w_forward(i) = wfor;
            obj.w_back(i) = wback;
        end
        
        function [v,w,err_x,err_y] = feedback(obj, actual_pose, desired_pose)
    
            kx = 0.0006;
            ky = 0.00008;
            

            error_pose = actual_pose - desired_pose;
            err_x_world = error_pose(1);
            err_y_world = error_pose(2);
            if norm(error_pose)<5
                ky = 0;
            end
            th = actual_pose(3);
            error_body = [cos(th) sin(th); -sin(th) cos(th)]*[err_x_world;err_y_world];
            up = [kx 0; 0 ky] * error_body;
            v = up(1);
            w = up(2);
            err_x = error_body(1);
            err_y = error_body(2);
     
        end
            
            
        function opVelocity(obj)
            global vl_real;      %left wheel velocity (actual)
            global vr_real;      %right wheel velocity (actual)
            global le;
            global re;
            global t_robot;
            x = 0;
            y = 0;
            th = 0;
            sref=0;
            t_prev=0;
            robotTrajObj = robotTrajectory();
            robotObj = robotModel();
            DistFinal = 0.3*12.565*robotTrajObj.Ks;
            tf = 2*robotTrajObj.t_pause + (DistFinal-(obj.vmax^2/obj.amax))/obj.vmax + 2*obj.vmax/obj.amax;
            i = 0;
            kdx = 0.0005;
            kdy = 0.0005;
            
            
            while(true)
                if i==0
                    tstart = tic;
                end
                i=i+1;
                t_now=toc(tstart);
                vfor = trapezoidalVelocityProfile(t_now, obj.amax, obj.vmax,1,tf,robotTrajObj.t_pause);
                dt = t_now-t_prev;
                
                wfor = getOmegaAtDistance(robotTrajObj,sref,obj.amax,obj.vmax,robotTrajObj.t_pause,tf);
                
                
                pose_des = getPoseAtDistance(robotTrajObj,sref,obj.amax,obj.vmax,robotTrajObj.t_pause,tf);
                pose_actual = [x,y,th];
                [vback,wback,errx,erry] = obj.feedback(pose_actual,pose_des);
                obj.setXY(x,y,i);
                obj.set_error(errx,erry,i);
                obj.set_vw(vfor,vback,wfor,wback,i);
                if i>1
                    vbackd = kdx*(obj.err_x(i-1)-obj.err_x(i));
                    wbackd = kdy*(obj.err_y(i-1)-obj.err_y(i));
                    obj.v_back = obj.v_back+vbackd;
                    obj.w_back = obj.w_back+wbackd;
                end
                
                
                
                V = obj.v_forward(i)+obj.f*obj.v_back(i);
                w = obj.w_forward(i)+obj.f*obj.w_back(i);
                
                %obj.set_vw(vfor,0,wfor,0,i);
                
                %V = obj.v_forward(i);
                %w = obj.w_forward(i);
                
                [vl,vr] = robotObj.VwTovlvr(V,w);
                [v_real,w_real] = lrtovw(vl_real,vr_real);
                th = th+0.5*w_real*dt;
                x = x+v_real*cos(th)*dt;
                y = y + v_real*sin(th)*dt;
                th = th+0.5*w_real*dt;
                
                if abs(V)<1e-5
                    obj.rob.sendVelocity(0,0);
                else
                    obj.rob.sendVelocity(vl,vr);
                end
                axis manual;
                axis equal;
                axis([-400 400 -400 400]);
                scatter(x,y,'.','b');
                hold on;
                %scatter(pose_des(1),pose_des(2),'.','r');
                %hold on;
                sref = sref + V*dt;
                if(t_now>=tf-robotTrajObj.t_pause)
                    obj.rob.sendVelocity(0,0);
                    break;
                end
                t_prev=t_now;
                pause(0.01);
            end
        end
    end
end
            