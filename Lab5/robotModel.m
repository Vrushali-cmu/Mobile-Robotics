classdef robotModel < handle
    properties(Constant)
        W = 0.235; %width of the robot (in m)
        W2 = 0.1175;  %width/2 of the robot (in m)
    end
    methods(Static=true)
        function [vl,vr] = VwTovlvr(V,w)
            vl = V-robotModel.W2*w;   %left wheel velocity
            vr = V+robotModel.W2*w;   %right wheel velocity
            
            %saturate absolute velocities at 0.3 m/s
            if abs(vl)>0.3
               vl = 0.3*vl/abs(vl);        
            end
            if abs(vr)>0.3
               vr = 0.3*vr/abs(vr);
            end
            
        end
        function [V w] = vlvrToVw(vl,vr)
            V = (vl+vr)/2;  %velocity
            w = (vr-vl)/robotModel.W;  %angular velocity
        end
    end
end