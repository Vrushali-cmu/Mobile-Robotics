x_arr = [];
y_arr = [];
th_arr = [];
x = 0;
y = 0;
th = 0;
k = 2;
t0 = 0;
dt = 0.001;
tf = 11.207*k;     %final time for cornu spiral
t = t0;
n = 0;
while t<tf
   vr = 1000*(0.1/k + 0.01174*t/(k^2));        
   vl = 1000*(0.1/k - 0.01174*t/(k^2));
   [v,w] = lrtovw(vl,vr);
   th = th+0.5*w*dt;
   th = th+0.5*w*dt;
   x = x+v*cos(th)*dt;
   y = y+v*sin(th)*dt;
   %if mod(n,10)==0
        x_arr = [x_arr;x];
        y_arr = [y_arr;y];
        th_arr = [th_arr;th];
   %end
   th = th+0.5*w*dt;
   t = t+dt;
   n = n+1;
end
   
plot(x_arr,y_arr);
   

