x_arr = [];
y_arr = [];
th_arr = [];
v_arr = [];
w_arr = [];
x = 0;
y = 0;
th = 0;
t0 = 0;
dt = 0.001;
t = t0;
ks = 0.5;
kv = 0.4;
tf = 12.565*ks/kv;  %final time for figure 8
n = 0;
figure(1);
xlim([-400 400]);
ylim([-400 400]);

while t<tf
   vr = 1000*(0.3*kv+0.14125*(kv/ks)*sin(t*kv/(2*ks)));        
   vl = 1000*(0.3*kv-0.14125*(kv/ks)*sin(t*kv/(2*ks)));
   [v,w] = lrtovw(vl,vr);
   th = th+0.5*w*dt;
   x = x+v*cos(th)*dt;
   y = y+v*sin(th)*dt;
   %if mod(n,10)==0
        x_arr = [x_arr;x];
        y_arr = [y_arr;y];
        th_arr = [th_arr;th];
        v_arr = [v_arr;vl];
        w_arr = [w_arr;w];
   %end
   
   %scatter(x,y,'.','b');
   %hold on
   %myPlot;
   th = th+0.5*w*dt;
   t = t+dt;
   n = n+1;
   %pause(0.001);
end
figure(1)   
plot(x_arr,y_arr);

figure(2)
plot(v_arr);
   

