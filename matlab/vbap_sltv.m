function [v1_c, r1_c,v2_c,r2_c] = vbap_sltv(USV1_ODOM,USV2_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;

% Convert to quaternions
quat1 = USV1_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(1);
quat2 = USV2_ODOM.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(1);

%Desired heading
teta1 = atan2(Yerr1,Xerr1);
teta2 = atan2(Yerr2,Xerr2);

%Errors
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(teta1-psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(teta2-psi2); 
kv = 0.1; kh = 3; 
v1_c = kv*distErr1;
r1_c = kh * headErr1; 
v2_c = kv*distErr2;
r2_c = kh * headErr2;
fprintf("USV1--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);

return
