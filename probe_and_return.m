%% First Run Probe and return
clear;clc;
% Start the foxbot node
% roslaunch foxbot foxbot.launch
% Start the force sensor node
% roslaunch netft_rdt_driver ft_sensor.launch

obj=foxbot();
% No work frames, all are tool frames
obj.setCRequest.FrameId=1;
obj.absCRequest.FrameId=1;
obj.getIKRequest.FrameId=1;

home_msg=obj.getCartesian();
home_pos=[home_msg.X;home_msg.Y;home_msg.Z];
home_ori=[home_msg.Q0;home_msg.Qx;home_msg.Qy;home_msg.Qz];
save organ;clc;

sub=rossubscriber('/ft_sensor/netft_data');
pause(0.2);
%% Second run
clear;load organ; clc;
obj=foxbot();% No work frames, all are tool frames
obj.setCRequest.FrameId=1;
obj.absCRequest.FrameId=1;
obj.getIKRequest.FrameId=1;

obj.goHome();

home_pos=[home_msg.X;home_msg.Y;home_msg.Z];
home_ori=[home_msg.Q0;home_msg.Qx;home_msg.Qy;home_msg.Qz];
sub=rossubscriber('/ft_sensor/netft_data');
pause(0.2);
%%
obj.moveCartesianFull(home_pos);
pause(0.2);
%%
% Height??
z_off=130; %130 bunny
pts=20;
l=150; %(in X direction)
b=140; % (in Y direction)
points=nan(pts,3);
planeZ=home_pos(3)+z_off;
%%
obj.moveCartesianFullOrientation(home_pos+[0;0;z_off],home_ori);
pause(1)
obj.moveCartesianFullOrientation(home_pos+[l;0;z_off],home_ori);
pause(1)
obj.moveCartesianFullOrientation(home_pos+[l;b;z_off],home_ori);
pause(1)
obj.moveCartesianFullOrientation(home_pos+[0;b;z_off],home_ori);
pause(1)
obj.moveCartesianFullOrientation(home_pos+[0;0;z_off],home_ori);
%%
%probedown(sub,obj,goal,origin,planeZ);
[p1,p2]=find_probe_points(l,b,z_off,pts);
i=1;
%%
for i=1:pts
    obj.moveCartesianFullOrientation(home_pos+[p1(i,1);p1(i,2);z_off],home_ori);
    points(i,:)=probedown(sub,obj,home_pos+[p2(i,1);p2(i,2);0],home_pos+p1(i,:)',planeZ);
    obj.moveCartesianFullOrientation(home_pos+[p1(i,1);p1(i,2);z_off],home_ori);
    if (isnan(points(i,1)))
        disp(['Point number ',num2str(i),' was skipped.']);
    else
        disp(['Point number ',num2str(i),' completed.']);
    end
end
obj.goHome();
save points.mat points;
  dx%%
figure;
scatter3(points(:,1),points(:,2),points(:,3));
axis equal;
view(129,49);
xlabel('X');ylabel('Y');zlabel('Z');