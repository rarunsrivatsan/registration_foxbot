%% Registration using foxbot
% The foxbot rotation is performed in absolute euler angles or rotation
% quaternions wrt the cartesian frame of reference.  Either provide a
% direction of rotation with a quaternion, or create by using eul2quat.
clear;clc;
obj=foxbot();
obj.goHome();
home_msg=obj.getCartesian();
home_pos=[home_msg.X;home_msg.Y;home_msg.Z];
home_ori=[home_msg.Q0;home_msg.Qx;home_msg.Qy;home_msg.Qz];
%% Straight down
startpt=home_pos+[0;0;-50];
obj.moveCartesianFullOrientation(startpt,[0;1;0;0]);
%% 45 degrees towards front is 0 -pi/4 pi
rz=0;ry=0;rx=pi;
quat=eul2quat([rz ry rx],'zyx');
r1=eul2rotm([rz ry rx]);

T=trvec2tform([0;0;60]');
T(1:3,1:3)=eye(3);
finalpt=T*[startpt;1];
obj.moveCartesianFullOrientation(finalpt(1:3),quat);
%%
r_step=eul2rotm([0 0 0.1]);
T(1:3,1:3)=r_step;
finalpt=T*[startpt;1];
% ERROR HERE
obj.moveCartesianFullOrientation(finalpt(1:3),rotm2quat(r_step));
%%
x=0;y=0;z=-1;
rx=0;
ry=asin(z/norm([x y z]));
rz=atan2(y,x);
r2=eul2rotm([rz ry rx]);
quat2=rotm2quat(r2*r1);
obj.moveCartesianFullOrientation(startpt,quat2);

%%
x=1;y=0;z=-1;
rx=0;
ry=asin(z/norm([x y z]));
rz=atan2(y,x);
quat2=quat.*eul2quat([rz ry rx]);
obj.moveCartesianFullOrientation(startpt,quat2);