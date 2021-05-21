function [v_c, r_c] = vbap_slsv(USV_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle
Xerr = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
Yerr = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
<<<<<<< HEAD

% Convert to quaternions
quat = USV_ODOM.Pose.Pose.Orientation; 
angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
psi = angles(1);

teta = atan2(Yerr,Xerr);

distErr = sqrt(Xerr^2 + Yerr^2); 
headErr = wrapToPi(teta-psi); 

kv = 0.1; kh = 3; 
v_c = kv*distErr;
r_c = kh * headErr; 

fprintf("Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta,psi,headErr,r_c,distErr,v_c);
=======
%psi = USV_ODOM.Twist.Twist.Angular.Z; 
teta = atan(Yerr/Xerr);

distErr = sqrt(Xerr^2 + Yerr^2); 
headErr = teta %- psi; 
wrappedHeadErr = wrapToPi(headErr); %Keep within pi to -pi

kv = 1; kh = 0.5; 


v_c = kv*distErr;
r_c = kh * wrappedHeadErr; 
>>>>>>> ddc44c000667f9f0105d81b0086af4bfd0c220f7
return
