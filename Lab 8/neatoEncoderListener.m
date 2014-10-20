

function neatoEncoderListener(event, handle)



global re;              %right encoder
global le;              %left encoder
global t_robot;               %timestamp
global vl_real;         %left wheel velocity
global vr_real;         %right wheel velocity

persistent prev_re;     %previous right encoder value
persistent prev_le;     %previous left encoder value
persistent prev_t;      %previous time value

re = event.data.right;
le = event.data.left;
t_robot = event.data.header.stamp.secs + (event.data.header.stamp.nsecs/1000000000.0);

%velocity  = difference in encoder value/difference in time stamp
vl_real = 0.001*(le-prev_le)/(t_robot-prev_t);       
vr_real = 0.001*(re-prev_re)/(t_robot-prev_t);


%assigns the previous encoder and time value for the next time this
%function is called
prev_re = re;
prev_le = le;
prev_t = t_robot;
end
