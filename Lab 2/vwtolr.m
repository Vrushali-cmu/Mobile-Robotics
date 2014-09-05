function [vl,vr] = vwtolr(v,w)
%Converts linear and angular velocity to left and right velocities
%{
if abs(v)>0.2
    v = 0.2*v/abs(v);
end
%}
d = 0.235;      %distance between robot wheels in m
vl = v-d*w/2;   %left wheel velocity
vr = v+d*w/2;   %right wheel velocity
%saturate absolute value of left and right velocitties at 0.3 m/s
if abs(vl)>0.3
    vl = 0.3*vl/abs(vl);        
end
if abs(vr)>0.3
    vr = 0.3*vr/abs(vr);
end
end