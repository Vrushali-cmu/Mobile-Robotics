range_arr = zeros(360,2);
img_count=1;
while(img_count<2)
i=1
    while(true)
    r = robot.laser.data.ranges(i)
    [x y b] = irToXy(i,r);
    %if(r>1 || r<0.05)
    %    x=0;
    %    y=0;
    %end
    
    range_arr(i,1) = x;
    range_arr(i,2) = y;
    
    i= i+1;
    
    if(i>360)
        break;
    end
    end
figure(img_count);
axis([-1 1 -1 1])
scatter(range_arr(:,2),range_arr(:,1),'.');
save('temp_file','range_arr');
beep;
pause(10);
img_count = img_count+1;
end
