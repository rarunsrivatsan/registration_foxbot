%% Creates a Foxbot class with movement commands
% Measurements in mm and degrees for cartesian and joint space
% respectively.
classdef foxbot
    %SetAccess = private, GetAccess = private
    properties(SetAccess = public)
        %         jogCClient
        %         jogCRequest
        
        absJClient
        absJRequest
        
        absCClient
        absCRequest
        absCResponse
        
        jogJClient
        jogJRequest
        jogJResponse
        
        stopClient
        stopRequest
        
        setCClient
        setCRequest
        
        getIKClient
        getIKRequest
        
        setSpeedClient
        setSpeedRequest
        
        isMovingSub
        
        pauseTime=0.05;
    end
    
    methods
        
        function obj=foxbot()
            % Create all the servers and request messages
            
            %             obj.jogCClient = rossvcclient('/foxbot/robot_JogCartesian');
            %             obj.jogCRequest = rosmessage(obj.jogCClient);
            
            obj.getIKClient = rossvcclient('/foxbot/robot_GetIK');
            obj.getIKRequest = rosmessage(obj.getIKClient);
            
            obj.absJClient = rossvcclient('/foxbot/robot_SetJoints');
            obj.absJRequest = rosmessage(obj.absJClient);
            
            obj.absCClient = rossvcclient('/foxbot/robot_GetCartesian');
            obj.absCRequest = rosmessage(obj.absCClient);
            
            obj.jogJClient = rossvcclient('/foxbot/robot_GetJoints');
            obj.jogJRequest = rosmessage(obj.jogJClient);
            
            obj.stopClient=rossvcclient('/foxbot/robot_Stop');
            obj.stopRequest=rosmessage(obj.stopClient);
            
            obj.setCClient = rossvcclient('/foxbot/robot_SetCartesian');
            obj.setCRequest = rosmessage(obj.setCClient);
            
            obj.setSpeedClient = rossvcclient('/foxbot/robot_SetSpeed');
            obj.setSpeedRequest = rosmessage(obj.setSpeedClient);
            
            obj.isMovingSub=rossubscriber('/robot_IsMoving');
        end
        
        function ret=isMoving(obj)
            resp=receive(obj.isMovingSub);
            ret=resp.Data;
        end
        
        function obj=setPauseTime(obj,t)
            % Override the set pause time
            obj.pauseTime(t);
        end
        
        function [resp]=getCartesian(obj)
            resp=call(obj.absCClient,obj.absCRequest);
        end
        
        function [resp]=getJoints(obj)
            resp=call(obj.jogJClient,obj.jogJRequest);
        end
        
        % Check this up
        %         function [resp]=moveCartesianDiff(obj,diff)
        %             % Move the foxbot in cartesian differential.
        %             obj.jogCRequest.X = diff(1);
        %             obj.jogCRequest.Y = diff(2);
        %             obj.jogCRequest.Z = diff(3);
        %             resp=call(obj.jogCClient, obj.jogCRequest);
        %             pause(obj.pauseTime);
        %         end
        
        function [resp]=moveJointsDiff(obj,diff)
            % Moves the foxbot to given joint angles absolute. Send difference values to
            % this function.
            obj.jogJResponse = call(obj.jogJClient,obj.jogJRequest);
            
            pos(1)=diff(1)+obj.jogJResponse.J1;
            pos(2)=diff(2)+obj.jogJResponse.J2;
            pos(3)=diff(3)+obj.jogJResponse.J3;
            pos(4)=diff(4)+obj.jogJResponse.J4;
            pos(5)=diff(5)+obj.jogJResponse.J5;
            pos(6)=diff(6)+obj.jogJResponse.J6;
            
            % Joints works in the opposite sense of Cartesian. It requires
            % absolute values therefore the current position was retrieved
            % and then the absolute calculated and sent.
            obj.absJRequest.J1= pos(1);
            obj.absJRequest.J2= pos(2);
            obj.absJRequest.J3= pos(3);
            obj.absJRequest.J4= pos(4);
            obj.absJRequest.J5= pos(5);
            obj.absJRequest.J6= pos(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=goHome(obj)
            % USE WITH CAUTION.
            % Moves the foxbot to home position
            goal=[0 0 90 0 90 180];
            obj.absJRequest.J1= goal(1);
            obj.absJRequest.J2= goal(2);
            obj.absJRequest.J3= goal(3);
            obj.absJRequest.J4= goal(4);
            obj.absJRequest.J5= goal(5);
            obj.absJRequest.J6= goal(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=moveJointsAbs(obj,goal)
            % USE WITH CAUTION.
            % Moves the foxbot to given joint angles absolute. Send absolute values to
            % this function.
            obj.absJRequest.J1= goal(1);
            obj.absJRequest.J2= goal(2);
            obj.absJRequest.J3= goal(3);
            obj.absJRequest.J4= goal(4);
            obj.absJRequest.J5= goal(5);
            obj.absJRequest.J6= goal(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        %         function [resp]=moveCartesianAbs(obj,goal)
        %             % USE WITH CAUTION.
        %
        %             % Move the foxbot in cartesian absolute. Cartesian can move
        %             % only in jog mode, therefore the current position is
        %             % subtracted to find the difference to jog and then the robot
        %             % is moved.
        %
        %             % Get current position by calling the client
        %             obj.absCResponse = call(obj.absCClient, obj.absCRequest);
        %
        %             % Calculate the difference and move ahead using usual jog
        %             diff.X = goal(1)-obj.absCResponse.X;
        %             diff.Y = goal(2)-obj.absCResponse.Y;
        %             diff.Z = goal(3)-obj.absCResponse.Z;
        %
        %             obj.jogCRequest.X = diff.X;
        %             obj.jogCRequest.Y = diff.Y;
        %             obj.jogCRequest.Z = diff.Z;
        %             resp=call(obj.jogCClient, obj.jogCRequest);
        %             pause(obj.pauseTime);
        %         end
        
        function [resp]=stopRobot(obj)
            resp=call(obj.stopClient,obj.stopRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=moveCartesianFull(obj,goal)
            % USE WITH CAUTION.
            if obj.setCRequest.FrameId==1
                q=[1 0 0 0];
            else
                q=[0 1 0 0];
            end
            
            if obj.checkIK(goal,q)
                % Move the foxbot in cartesian absolute. Cartesian can move
                % only in jog mode, therefore the current position is
                % subtracted to find the difference to jog and then the robot
                % is moved.
                obj.setCRequest.X = goal(1);
                obj.setCRequest.Y = goal(2);
                obj.setCRequest.Z = goal(3);
                obj.setCRequest.Q0 = q(1);
                obj.setCRequest.Qx = q(2);
                obj.setCRequest.Qy = q(3);
                obj.setCRequest.Qz = q(4);
                resp=call(obj.setCClient, obj.setCRequest);
                
                resp=resp.Ret;
                pause(obj.pauseTime);
            else
                resp=0;
            end
        end
        
        function [resp]=moveCartesianFullOrientation(obj,goal,quat)
            % USE WITH CAUTION.
            if obj.checkIK(goal,quat)
                % Move the foxbot in cartesian absolute. Cartesian can move
                % only in jog mode, therefore the current position is
                % subtracted to find the difference to jog and then the robot
                % is moved. Quaternion of endfactor is specified with:
                % quat = [w x y z]
                obj.setCRequest.X = goal(1);
                obj.setCRequest.Y = goal(2);
                obj.setCRequest.Z = goal(3);
                
                obj.setCRequest.Q0 = quat(1);
                obj.setCRequest.Qx = quat(2);
                obj.setCRequest.Qy = quat(3);
                obj.setCRequest.Qz = quat(4);
                
                resp=call(obj.setCClient, obj.setCRequest);
                resp=resp.Ret;
                pause(obj.pauseTime);
            else
                resp=0;
            end
        end
        
        % Check whether the point is within workspace
        function [ret]=checkIK(obj,pos,quat)
            
            obj.getIKRequest.X=pos(1);
            obj.getIKRequest.Y=pos(2);
            obj.getIKRequest.Z=pos(3);
            
            obj.getIKRequest.Q0=quat(1);
            obj.getIKRequest.Qx=quat(2);
            obj.getIKRequest.Qy=quat(3);
            obj.getIKRequest.Qz=quat(4);
            
            resp=call(obj.getIKClient, obj.getIKRequest);
            ret=resp.Ret;
        end
        
        % Abort procedure DOES NOT MOVE IN TOOL FRAME RIGHT NOW! FIX THIS
        %         function abort_exp(obj,step)
        %             curr_pos_msg=obj.getCartesian();
        %             curr_pos=[curr_pos_msg.X curr_pos_msg.Y curr_pos_msg.Z];
        %             curr_ori=[curr_pos_msg.Q0 curr_pos_msg.Qx curr_pos_msg.Qy curr_pos_msg.Qz];
        %             obj.moveCartesianFullOrientation(curr_pos+[step -step step],curr_ori);
        %         end
        function [ret]=changeSpeed(obj,speed)
            obj.setSpeedRequest.Tcp=speed;
            obj.setSpeedRequest.Ori=speed;
            
            resp=call(obj.setSpeedClient, obj.setSpeedRequest);
            ret=resp.Ret;
        end
    end
end