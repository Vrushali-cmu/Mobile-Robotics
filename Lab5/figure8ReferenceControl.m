classdef figure8ReferenceControl < handle
    
    properties (GetAccess = 'public', SetAccess = 'private')
        v
        w
        Ks
        Kv
        tPause
    end
    
    methods (Static)
        function [v,w] = lrtovw(vl,vr)
        d = 0.235;        %distance between the wheels
        v = 0.5*(vl+vr);
        w = (vr-vl)/d;
        end
        function [vl,vr] = vwtolr(v,w)
            %Converts linear and angular velocity to left and right velocities
            d = 0.235;      %distance between robot wheels in m
            vl = v-d*w/2;   %left wheel velocity
            vr = v+d*w/2;   %right wheel velocity
  
            if abs(vl)>0.3
                vl = 0.3*vl/abs(vl);        
            end
            if abs(vr)>0.3
                vr = 0.3*vr/abs(vr);
            end
        end
    end
    
    methods
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
            obj.Ks = Ks;
            obj.Kv = Kv;
            obj.tPause = tPause;
            %{
            while true
                timeNow = toc(tstart);
                [obj.v, obj.w] = computeControl(obj,timeNow);
                if timeNow>getTrajectoryDuration(obj)
                    break;
                end
            end
            %}
        end
        
        function obj = computeControl(obj,timeNow)
            if timeNow<obj.tPause
                obj.v = 0;
                obj.w = 0;
            elseif timeNow<getTrajectoryDuration(obj)-obj.tPause
                vr = 0.3*obj.Kv + 0.14125*(obj.Kv/obj.Ks)*sin(timeNow*obj.Kv/(2*obj.Ks));
                vl = 0.3*obj.Kv - 0.14125*(obj.Kv/obj.Ks)*sin(timeNow*obj.Kv/(2*obj.Ks));
                [obj.v,obj.w] = lrtovw(vl,vr);
            else
                obj.v = 0;
                obj.w =0;
            end
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = 12.565*obj.Ks/obj.Kv;
        end
    end
end
        
            
            
                
                
                
            