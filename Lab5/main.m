global re;              %right encoder
global le;              %left encoder
global t_robot;               %timestamp
global vl_real;              %left wheel velocity
global vr_real;              %right wheel velocity

robot_name = 'femto';
feedback = 1;
traj = trajectoryFollower(robot_name,feedback);
lh = event.listener(traj.rob.encoders,'OnMessageReceived',@neatoEncoderListener);
traj.opVelocity;

