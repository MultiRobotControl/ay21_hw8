function [u_c, r_c] = vbap_slsv(USV_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle

% Get odom data from CORA
u = USV_ODOM.Twist.Twist.Linear.X;    % CORA Surge Velocity [m/s]
v = USV_ODOM.Twist.Twist.Linear.Y;    % CORA Sway Velocity [m/s]
q = [USV_ODOM.Pose.Pose.Orientation.W,USV_ODOM.Pose.Pose.Orientation.X,...
    USV_ODOM.Pose.Pose.Orientation.Y,USV_ODOM.Pose.Pose.Orientation.Z];
e = quat2eul(q);
psi = e(:,1); % CORA Yaw [rad]

% Determine Distance between CORA and rabbit
dist2rabbit_x = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
dist2rabbit_y = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;

% Relative steering angle from CORA to rabbit
psi_error = wrapToPi(atan2(dist2rabbit_y,dist2rabbit_x)-psi);
    
% control gains
k_v = 0.25; % Surge Velocity gain
k_h = 4; % Steering gain

% control law 
u_c = k_v * sqrt(dist2rabbit_x^2 +dist2rabbit_y^2);  % Surge Velocity Command
r_c = k_h * psi_error;        % Yaw Rate Command i.e Turn Rate Cmd


return
