load('range_img_3.mat');
img = rangeImage(range_arr,1,0);
img_corrected = rangeImage(range_arr,1,1);

figure(1);
img.plotXvsY();
hold on;
img.findLineCandidate(img_corrected.tArray(4));

figure(2);
img_corrected.plotXvsY();

