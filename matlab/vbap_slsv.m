function [v_c, r_c] = vbap_slsv(USV_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle
Xerr = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
Yerr = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
%psi = USV_ODOM.Twist.Twist.Angular.Z; 
teta = atan(Yerr/Xerr);

distErr = sqrt(Xerr^2 + Yerr^2); 
headErr = teta %- psi; 
wrappedHeadErr = wrapToPi(headErr); %Keep within pi to -pi

kv = 1; kh = 0.5; 


v_c = kv*distErr;
r_c = kh * wrappedHeadErr; 
return
