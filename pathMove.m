% STARTED AT 03:27
clear;clc; close all;
%%
% find p1,p2,p3,p4
% ______________
% |p4         p3|
% |             |
% |             |
% |p1         p2|
% |_____________|
xoff=0;
yoff=0;
zoff=100;
p1=[537.17+xoff -88.49+yoff 84.43+zoff]';
p2=[537.17+xoff -1.96+yoff 84.03+zoff]';
p3=[451.66+xoff -1.96+yoff 83.08+zoff]';
p4=[451.66+xoff -90.61+yoff 83.11+zoff]';
% 
% p1=[490.12+xoff -53.61+yoff 76.08+zoff]';
% p2=[490.12+xoff 36.25+yoff 76.08+zoff]';
% p3=[402.06+xoff 33.25+yoff 76.68+zoff]';
% p4=[402.06+xoff -50.61+yoff 74.18+zoff]';
%%
sub=rossubscriber('/ft_sensor/netft_data');
pause(0.2);
obj=foxbot();
%%
obj.moveCartesianFull(p1');
%%
% find z_mean
centroid=mean([p1,p2,p3,p4]')';
zmean=centroid(3);

%find zmax,zmin
zmax=zmean+2;%assuming units are in mm
zmin=zmean-2;

[l,indx1]=min([norm(p2(1:2)-p1(1:2)),norm(p3(1:2)-p4(1:2))]);
[b,indx2]=min([norm(p4(1:2)-p1(1:2)),norm(p3(1:2)-p2(1:2))]);

ldir=p2(1:2)-p1(1:2);

ldir=ldir/norm(ldir);

bdir=p4(1:2)-p1(1:2);

bdir=bdir/norm(bdir);

result=[];

%%
% add plot function
num_intervals=15;
x=0:l/num_intervals:l;
y=0:b/num_intervals:b;
[X,Y]=meshgrid(x,y);
Z=0*X;
force=Z;
%%
figure
axis equal;
h1=surf(X,Y,Z);
shading interp;
colormap hot;
colorbar
xlabel('X Axis');
ylabel('Y Axis');
view(0,90);
title('Spatial stiffness mapping');
%%
for j=1:num_intervals+1
    for i=1:num_intervals+1
        
        p=p1(1:2)+i*l/(num_intervals+1)*ldir+j*b/(num_intervals+1)*bdir;
        
        resp=atiThreshold(sub,obj,[p(1) p(2) zmin]);
        forcemat{j,i,:}=resp{1};
        force(j,i)=forcemat{j,i}(end);
        stiffness=abs(force(j,i))/(zmean-resp{2}.Z);
        Z(j,i)=stiffness;
        
        %save result{i}=[px,py,stiffness]
        result{j,i}=[x(i),y(j),stiffness/1000];
        
        %Update plot with new stiffness
        h1.CData=Z;
        axis equal;
    end
end
obj.goHome;
save('expt.mat')