function pose_f = GetObstaclePose(range_data)

range_arr = zeros(360,2);

i=1
    while(true)
        r = range_data(i);
        [x y b] = irToXy(i,r);
    
        range_arr(i,1) = x;
        range_arr(i,2) = y;
    
        i= i+1;
    
        if(i>360)
            break;
        end
    end


img = rangeImage(range_arr,1,0);
img_corrected = rangeImage(range_arr,1,1);

%figure(1);
%img.plotXvsY();
%hold on;

[l_err_min num_f pose_f] = img.findLineCandidate(img_corrected.tArray(1));
i=2;
while(i<img_corrected.numPix+1)
    [l_err_tmp num_tmp pose_tmp] = img.findLineCandidate(img_corrected.tArray(i));
    if(num_tmp>2)
        if (l_err_tmp<l_err_min)
            l_err_min = l_err_tmp;
            num_f = num_tmp;
            pose_f = pose_tmp;
        end
    end
    i=i+1;
end
%plot(pose_f(1,2),pose_f(1,1),'+');
%hold on;
%x = [pose_f(2)-pose_f(3) pose_f(1)+1];
%y = [pose_f(2)+pose_f(3) pose_f(1)-1];
%plot(y,x);


%figure(2);
%img_corrected.plotXvsY();

