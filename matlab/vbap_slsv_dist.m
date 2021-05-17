function [u_c, r_c] = vbap_slsv(usv_odom, rabbit_position)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle
    
% Rabbit - Virtual leader
xl = rabbit_position.Point.X;
yl = rabbit_position.Point.Y;

% USV State
x = usv_odom.Pose.Pose.Position.X;
y = usv_odom.Pose.Pose.Position.Y;
u = sqrt(usv_odom.Twist.Twist.Linear.X^2 + usv_odom.Twist.Twist.Linear.Y^2);
q = [usv_odom.Pose.Pose.Orientation.W, usv_odom.Pose.Pose.Orientation.X, ...
    usv_odom.Pose.Pose.Orientation.Y, usv_odom.Pose.Pose.Orientation.Z];
e = quat2eul(q);
psi = e(1);

% Distance to leader
dx = xl-x;
dy = yl-y;
dist = sqrt( dx^2 + dy^2);
% Angle to leader
a2l = atan2(dy, dx);

% Angle Error
aerr = a2l - psi;
% Clamp - ensure that the error is (-pi,pi]
if aerr > pi
    aerr = aerr - 2*pi;
end
if aerr < -pi
    aerr = aerr + 2*pi;
end

% Gains
kv = 0.1;
kh = 10.0;
u_c = kv*dist;
r_c = kh*aerr;

thresh = 5.0;
if dist < thresh
    u_c = 0.0;
end



% For debugging
fprintf("Distance=%5.1f m, AngleError=%6.2f rad, SurgeCmd=%5.2f m/s, YawRateCmd=%6.2f rad/s\n", ...
    dist, aerr, u_c, r_c);
return
