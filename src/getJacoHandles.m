function [joint_handles,jaco_handle,JacoHand_Dummy7] = getJacoHandles(clientID,vrep,ArmJoints)

%====================================================================
% Get Jaco handles
%====================================================================
% Get "handle" to joints of robot to move the Jaco arm
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        sprintf('could not get joint handle%d = %s', i,ArmJoints(11*i+1:11*i+11))
    end
end
%%% Get the handle of Jaco object
[result, jaco_handle] = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco handle')
end
%%% Get the handle of end effector (dummy7 objec)
[result, JacoHand_Dummy7] = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 handle')
end


end