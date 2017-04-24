function result=atiThreshold(sub,foxbotObj,goal)
% Stops the foxbot when force exceeds a limit and then restarts its
% original path

% Force in Grams
fthresh=-2;
dthresh=3; %in mm
dincr=0.2; %in mm
% Go over the point to probe

% WARNING MOVED TO GOAL WITH +2 mm depth indentation. Change in next line.

foxbotObj.moveCartesianFull([goal(1) goal(2) goal(3)+dthresh]);
pause(0.2);

% Receive the sensor readings
forceMsg=receive(sub);
forceData=forceMsg.Wrench.Force.Z;
posMsg=foxbotObj.getCartesian();
distance=posMsg.Z-goal(3);
%%
i=2;
% If both force and position are not reached
while(forceData(end)>fthresh && distance>0)
    
    forceMsg=receive(sub);
    posMsg=foxbotObj.getCartesian();
    distance=posMsg.Z-goal(3);
    
    % Move towards it
    foxbotObj.moveCartesianFull([goal(1) goal(2) posMsg.Z-dincr]);
    
    % Receive sensor values and then recheck.
    forceMsg=receive(sub);
    forceData(i)=forceMsg.Wrench.Force.Z;
    posMsg=foxbotObj.getCartesian();
    distance=posMsg.Z-goal(3);
    i=i+1;
end
result={forceData;posMsg};

end