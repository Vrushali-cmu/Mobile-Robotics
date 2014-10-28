function [x, y, b] = irToXy(pts)
%b is the bearing
b = zeros(1,360);
b(1:180) = 1:180;
b(181:360) = -179:1:0;
x = pts.*cos(b*pi/180);
y = pts.*sin(b*pi/180);


end
