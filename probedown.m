function result=probedown(sub,foxbotObj,goal,origin,planeZ)
% Stops the foxbot when force exceeds a limit and then restarts its
% original path

% Force in CentiGrams
fthresh=-0.15;
dincr=0.15; %in mm

% % Go over the point to probe,
% [~]=foxbotObj.moveCartesianFull([origin(1) origin(2) planeZ]);

% Rotate and move in the direction to probe
v=goal-origin;

%v=origin-goal;
u=v./norm(v);
z_down=[0;0;-1];
axis_rot=cross(z_down,u);
theta=acos(z_down'*u);
quat=[cos(theta/2) axis_rot(1)*sin(theta/2) axis_rot(2)*sin(theta/2) axis_rot(3)*sin(theta/2)];

if ~foxbotObj.moveCartesianFullOrientation([origin(1) origin(2) planeZ],quat)
    disp('Skipping a point');
    result=[nan nan nan];
    return;
end


if ~foxbotObj.moveCartesianFullOrientation(origin,quat)
    disp('Skipping a point');
    result=[nan nan nan];
    return;
end

div=100;

% Receive the sensor readings
forceMsg=receive(sub);
forceData=forceMsg.Wrench.Force.Z;
%%
i=2;
% If both force and position are not reached
while((forceData(end)-forceData(1))>fthresh && i<100)
    %     posMsg=foxbotObj.getCartesian();
    %
    %     % Move towards it
    %     foxbotObj.moveCartesianFull([goal(1) goal(2) posMsg.Z-dincr]);
    %
    newpt=origin+u*i*norm(v)/div;
    
    if ~foxbotObj.moveCartesianFullOrientation(newpt,quat)
        disp('Skipping a point');
        result=[nan nan nan];
        return;
    end
    
    % Receive sensor values and then recheck.
    forceMsg=receive(sub);
    forceData(i)=forceMsg.Wrench.Force.Z;
    %forceData(i)=sqrt((forceMsg.Wrench.Force.Z)^2+(forceMsg.Wrench.Force.X)^2+(forceMsg.Wrench.Force.Y)^2);
    %disp(forceData(i))
    i=i+1;
end

if i==100
    result=[nan nan nan];
    disp('This point was not registered');
else
    posMsg=foxbotObj.getCartesian();
    result=[posMsg.X posMsg.Y posMsg.Z];
end

if ~foxbotObj.moveCartesianFullOrientation([origin(1) origin(2) origin(3)],quat)
    disp('Error in returning');
    result=[nan nan nan];
    return;
end

if ~foxbotObj.moveCartesianFull([origin(1) origin(2) planeZ])
    disp('Error in returning');
    result=[nan nan nan];
    return;
end

end