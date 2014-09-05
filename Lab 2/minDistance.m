function [i,r] = minDistance(range)
%Gives index and value of minimum distance between 25 cm and 2 m
%Returns [0,0] if no values exist between 0.25-2m
view = [range(1:45) range(316:360)];    %scans only between +-45 degrees
view(view<0.25) = 10;                   %change all values less than 0.25 to 10
i = find(view==min(view),1);            %find first index of min value
r = min(view);                          %min distance
if(i>45)
    i=270+i;
end
if(r>2)
    r=0;
    i = 0;
end

end