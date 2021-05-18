clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive an odometry messge, save it here.
global CORA1_ODOM;  
global CORA2_ODOM;  

% When we receive a rabbit posiition, save it here
global RABBIT_POSITION;

% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Subscribers
usv_sub1 = rossubscriber('cora1/cora/sensors/p3d',@cora_odom_callback, ...
    'DataFormat', 'struct');
usv_sub2 = rossubscriber('cora2/cora/sensors/p3d',@cora_odom_callback, ...
    'DataFormat', 'struct');
% Add another subscriber here for the rabbit!
rabbit_sub = rossubscriber('/rabbit',@rabbit_callback,...
    'DataFormat', 'struct');

% Setup Publisher
cmd_pub1 = rospublisher('cora1/cora/cmd_vel','geometry_msgs/Twist');
cmd_pub2 = rospublisher('cora2/cora/cmd_vel','geometry_msgs/Twist');
cmd_msg = rosmessage(cmd_pub1);

% Infinite loop
while true
    % Call a function to implement the VBAP algorithm.
    u1 = 0.0;
    r1 = 0.0;
    u2 = 0;
    r2 = 0;
    if isempty(RABBIT_POSITION) 
        disp('WARNING - Rabbit position is empty')
    elseif isempty(CORA1_ODOM) 
        disp('WARNING - CORA1 odometry is empty')
    elseif isempty(CORA2_ODOM) 
        disp('WARNING - CORA2 odometry is empty')
    else
        [u1, r1] = vbap_slsv_dist(CORA1_ODOM, RABBIT_POSITION);
        [u2, r2] = vbap_slsv_dist(CORA2_ODOM, RABBIT_POSITION);
    end
    
    
    % Publish the results
    cmd_msg.Linear.X = u1;
    cmd_msg.Angular.Z = r1;
    send(cmd_pub1, cmd_msg);
    cmd_msg.Linear.X = u2;
    cmd_msg.Angular.Z = r2;
    send(cmd_pub2, cmd_msg);
    
    pause(0.1);
end

    

