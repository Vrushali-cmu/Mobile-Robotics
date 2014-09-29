

function neatoEncoderEventListener(event, handle)



global re;              %right encoder
global le;              %left encoder
global t;               %timestamp
global vl;              %left wheel velocity
global vr;              %right wheel velocity
persistent prev_re;     %previous right encoder value
persistent prev_le;     %previous left encoder value
persistent prev_t;      %previous time value

re = event.data.right;
le = event.data.left;
t = event.data.header.stamp.secs + (event.data.header.stamp.nsecs/1000000000.0);

%velocity  = difference in encoder value/difference in time stamp
vl = (le-prev_le)/(t-prev_t);       
vr = (re-prev_re)/(t-prev_t);

%assigns the previous encoder and time value for the next time this
%function is called
prev_re = re;
prev_le = le;
prev_t = t;
end
