function pose_f = GetObstaclePose(range_data)
offset = 0.20;
range_arr = zeros(360,2);

i=1;
    while(true)
        r = range_data(i);
        [x,y,~] = irToXy(i,r);
    
        range_arr(i,1) = x;
        range_arr(i,2) = y;
    
        i= i+1;
    
        if(i>360)
            break;
        end
    end


img = rangeImage(range_arr,1,0);
img_corrected = rangeImage(range_arr,1,1);

figure(10);
img.plotXvsY();
hold on;
figure(1)
[l_err_min num_f pose_f] = img.findLineCandidate(img_corrected.tArray(1));
i=2;
while(i<img_corrected.numPix+1)
    [l_err_tmp, num_tmp, pose_tmp] = img.findLineCandidate(img_corrected.tArray(i));
    if(num_tmp>4)
        if (l_err_tmp<l_err_min)
            l_err_min = l_err_tmp;
            num_f = num_tmp;
            pose_f = pose_tmp;
        end
    end
    i=i+1;
end

x = pose_f(1);
y = pose_f(2);
obslope = atan(pose_f(3));
plot([x-0.625*cos(obslope)  x+0.625*cos(obslope)], [y-0.625*sin(obslope)  y+0.625*sin(obslope)]);
hold on
sl = atan(-1/pose_f(3));
if x<0
    offset = 0.08;
end

if x<0 && y<0
    sl = sl+pi;
elseif x<0 && y>0
    if sl<0
        sl = sl+pi;
    end
elseif x>0 && y<0
    %sl = -abs(sl);
end

x1 = x-offset*cos(sl);
y1 = y-offset*sin(sl);
pose_f = [x1 y1 sl];
%{
x2 = x-offset*cos(sl);
y2 = y-offset*sin(sl);

d1 = sqrt(x1^2+y1^2);
d2 = sqrt(x2^2+y2^2);
if d1<d2
    pose_f = [x1 y1 sl];
else
    pose_f = [x2 y2 sl];
end
%}



%{
x = [pose_f(2)-pose_f(3) pose_f(1)+1];
y = [pose_f(2)+pose_f(3) pose_f(1)-1];
plot(y,x);
%}

%figure(2);
%img_corrected.plotXvsY();
%hold on
%axis([-2 2 -2 2]);

