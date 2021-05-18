function cora_odom_callback(topic, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV Odometry')

% Declare global variables to store odometry message
global CORA1_ODOM;
disp(topic)

CORA1_ODOM = msg;
