function [p1,p2 ] = find_probe_points( lx,ly,lz,N )

vertices=[0,0,0;lx,0,0;0,ly,0;lx,ly,0;0,0,lz;lx,0,lz;0,ly,lz;lx,ly,lz];

centroid=mean(vertices);

points=[];
tt=1;
for ii=0.1*lx:0.9*lx
    for jj=0.1*ly:0.9*ly
        points{1}(tt,:)=[ii,jj,lz];tt=tt+1;%top
    end
end

tt=1;
for ii=0.1*lx:0.9*lx
    for jj=0.1*lz:0.9*lz
        points{4}(tt,:)=[ii,0,jj];tt=tt+1; %xz,y=0
    end
end

tt=1;
for ii=0.1*lx:0.9*lx
    for jj=0.1*lz:0.9*lz
        points{2}(tt,:)=[ii,ly,jj];tt=tt+1;%xz ,y=ly
    end
end
tt=1;
for ii=0.1*ly:0.9*ly
    for jj=0.1*lz:0.9*lz
        points{3}(tt,:)=[0,ii,jj];tt=tt+1;%yz plane, x=0'
    end
end


tt=1;
for ii=0.1*ly:0.9*ly
    for jj=0.1*lz:0.9*lz
        points{5}(tt,:)=[lx,ii,jj];tt=tt+1;%yz plane x=lx
    end
end
tt=1;p1=[];


for ii=1:ceil(N/5)
    for jj=1:5
        tmp=randperm(size(points{jj},1));idx=tmp(1);
        p1(tt,:)=points{jj}(idx,:);
        p2(tt,:)=centroid;
        tt=tt+1;
    end
end


end

