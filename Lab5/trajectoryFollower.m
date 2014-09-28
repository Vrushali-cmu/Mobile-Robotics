classdef trajectoryFollower < handle
    properties
        vmax = 0.25;
        amax = 3*vmax;
    end
    methods
        function opVelocity()
            sref=0;
            t_prev=0;
            robotTrajObj = robotTrajectory();
            robotObj = robotModel();
            DistFinal = 0.3*12.565*robotTrajObj.Ks;
            tf = 2*tPause + DistFinal/vmax + vmax/amax;
            
            tstart = tic();
            while(true)
                t_now=toc(tstart);
                V = trapezoidalVelocityProfile(t_now, obj.amax, obj.vmax,1,tf,robotTrajObj.tPause);
                dt = t_now-t_prev;
                sref = sref + V*dt;
                w = getOmegaAtDistance(robotTrajObj,sref);
                [vl,vr] = robotObj.VwTovlvr(V,w);
                robot.sendVelocity(vl,vr);
                if(t_now>=obj.tf)
                    robot.sendVeloctiy(0,0);
                    break;
                end
                t_prev=t_now;
            end
        end
    end
end
            