%converts left and right wheel velocity to v and omega

function [v,w] = lrtovw(vl,vr)
d = 0.235;        %distance between the wheels
v = 0.5*(vl+vr);
w = (vr-vl)/d;
end