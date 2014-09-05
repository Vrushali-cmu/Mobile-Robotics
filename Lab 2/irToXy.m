function [x y b] = irToXy(i,r)
%b is the bearing
if i>180
    b = -360+i;
else
    b = i;
end
x = r*cos(b*pi/180);
y = r*sin(b*pi/180);
end
