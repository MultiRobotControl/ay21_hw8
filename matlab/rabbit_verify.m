%% ME4823 Assignment 7
%% Load .bag file and create bag file object. 

fname = '2021-05-22-15-35-19.bag'; 
bag = rosbag(fname);
 
% Display available topics and message types in bag file. 
bag.AvailableTopics

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

% Create a time series of the Cmd data
% Retrieve the messages as a cell array
cmd_msgs = select(bag,'Topic','/cora/cmd_vel');
 
cmd_ts = timeseries(cmd_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
rabbit_msgs = select(bag,'Topic','/rabbit');
 
% Create a timeseries object of the subset of message fields we are interested in
rabbit_ts = timeseries(rabbit_msgs,'Point.X','Point.Y','Point.Z');
%% 1) Plot X-Y Position of USV

figure(1); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom_ts.Data(:,1),odom_ts.Data(:,2),'LineWidth', 2); hold on
plot(rabbit_ts.Data(:,1),rabbit_ts.Data(:,2),'r','LineWidth', 1); 
title('(1) X-Y Position of USV and Rabbit')
legend('USV Position', 'Rabbit Position', 'Location','best')
xlabel('X')
ylabel('Y')
grid on

%% 2) Plots of X vs time and Y vs time of USV

figure(2); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
subplot(2,1,1)
plot(odom_ts.Time, odom_ts.Data(:,1)); hold on
plot(rabbit_ts.Time, rabbit_ts.Data(:,1));
title('(2.a) X Position vs Time')
legend('USV Position', 'Rabbit Position', 'Location','best')
xlabel('Time')
ylabel('X')
grid on

subplot(2,1,2)
plot(odom_ts.Time,odom_ts.Data(:,2)); hold on
plot(rabbit_ts.Time, rabbit_ts.Data(:,2));
title('(2.b) Y Position vs Time')
legend('USV Position', 'Rabbit Position', 'Location','best')
title('(2.a) X Position vs Time of USV')
xlabel('Time')
ylabel('X')
grid on