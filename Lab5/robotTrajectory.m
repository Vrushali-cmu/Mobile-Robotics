classdef robotTrajectory < handle
   properties
      distance
      velocity
      poses
      time
      omega
      ks = 0.5;
      kv = 0.5;
      t_pause = 0.5;
      num_samples = 1000;
   end
   methods
        function obj = robotTrajectory()
            %robot_obj = robotModel();
            velocity_obj = figure8ReferenceControl(ks,kv,t_pause);
            tf = velocity_obj.getTrajectoryDuration();
            obj.distance = zeros(obj.num_samples,1);
            obj.velocity = zeros(obj.num_samples,1);
            obj.omega = zeros(obj.num_samples,1);
            obj.poses = zeros(obj.num_samples,3);
            dt = tf/obj.num_samples;
            obj.time = [1:obj.num_samples]*dt;
            for i=2:obj.num_samples
                [obj.velocity(i),obj.omega(i)] = velocity_obj.computeControl(dt*(i-1));
                obj.distance(i) = obj.distance(i-1) + obj.velocity(i-1)*dt;
                temp_theta = obj.poses(i-1,3)+obj.omega(i-1)*dt/2;
                obj.poses(i,1) = obj.poses(i-1,1)+obj.velocity(i-1)*cos(temp_theta)*dt;
                obj.poses(i,2) = obj.poses(i-1,2)+obj.velocity(i-1)*sin(temp_theta)*dt;
                obj.poses(i,3) = temp_theta+obj.omega(i-1)*dt/2;
            end
        end 
        %{
       function velocity = getVelocityAtTime(obj, t)
           for time=0:dt:t
              obj.V = (vr+vl)/2 
              dist = dist + velocity * dt;
           end
       end
       function distance = getVelocityAttime(obj, t)
            for time=0:dt:t
                 obj.w = (vr-vl)/2 ;   
            end
       end
       function pose = getPosesAtAttime(obj,t)
           pose = pose()           
       end
       %}
       
   end
end