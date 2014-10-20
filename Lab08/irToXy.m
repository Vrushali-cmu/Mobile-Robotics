function [x y b] = irToXy(i,r)
%b is the bearing
b = (i>180)*(-360) + i %if i>180, then b=i-360, else b=i
x = r*cos(b*pi/180);
y = r*sin(b*pi/180);
end
