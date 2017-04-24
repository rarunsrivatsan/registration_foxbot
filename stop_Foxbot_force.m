% Stops the foxbot when force exceeds a limit and then restarts its
% original path
clear;clc;

optoSub=rossubscriber('/WrenchData');
isMovingSub=rossubscriber('/robot_IsMoving');

pause(0.1);

bias=optoSub.LatestMessage.Wrench.Force;
foxbotObj=foxbot();

% First move to joint 0 0 0
foxbotObj.goHome();
interrupted=0;
%%
while(1)
    foxbotObj.moveCartesianAbs([320 0 300]);
    
    msg=receive(isMovingSub);
    while msg.Data
        [~,mag]=getOptoForce(optoSub,bias);
        
        if mag>0.3
            foxbotObj.stopRobot();
            pos=foxbotObj.getCartesian();
            interrupted=1;
        end
        
        while(mag>0.3)
            [~,mag]=getOptoForce(optoSub,bias);
            pause(0.01);
        end
        
        if interrupted
            foxbotObj.moveCartesianAbs([320 0 300]);
            interrupted=0;
        end
        
        msg=receive(isMovingSub);
    end
    
    foxbotObj.moveCartesianAbs([320 0 500]);
    
    msg=receive(isMovingSub);
    while msg.Data
        [~,mag]=getOptoForce(optoSub,bias);
        
        if mag>0.3
            foxbotObj.stopRobot();
            pos=foxbotObj.getCartesian();
            interrupted=1;
        end
        
        while(mag>0.3)
            [~,mag]=getOptoForce(optoSub,bias);
            pause(0.01);
        end
        
        if interrupted
            foxbotObj.moveCartesianAbs([320 0 500]);
            interrupted=0;
        end
        msg=receive(isMovingSub);
    end
    
end