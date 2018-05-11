function thetas = thetaStart(clientID,vrep,joint_handles)

%%% Get the current value of the current joint2 for checking limits
[result, theta1] = vrep.simxGetJointPosition(clientID, joint_handles(1), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 1)
end
%%% Get the current value of the current joint3 for checking limits
[result, theta2] = vrep.simxGetJointPosition(clientID, joint_handles(2), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 2)
end
%%% Get the current value of the current joint2 for checking limits
[result, theta3] = vrep.simxGetJointPosition(clientID, joint_handles(3), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 3)
end
%%% Get the current value of the current joint3 for checking limits
[result, theta4] = vrep.simxGetJointPosition(clientID, joint_handles(4), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 4)
end
%%% Get the current value of the current joint2 for checking limits
[result, theta5] = vrep.simxGetJointPosition(clientID, joint_handles(5), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 5)
end
%%% Get the current value of the current joint3 for checking limits
[result, theta6] = vrep.simxGetJointPosition(clientID, joint_handles(6), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', 6)
end

thetas = [theta1; theta2; theta3; theta4; theta5; theta6];

end