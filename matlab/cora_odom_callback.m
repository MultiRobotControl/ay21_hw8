function cora_odom_callback(src, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV Odometry')

% Declare global variables to store odometry message
global CORA1_ODOM;
global CORA2_ODOM;

if contains(src.TopicName, 'cora1')
    CORA1_ODOM = msg;
elseif contains(src.TopicName, 'cora2')
    CORA2_ODOM = msg;
else
    disp("Warning - callback doesn't know what to do with TopicName:")
    disp(src.TopicName)
end
