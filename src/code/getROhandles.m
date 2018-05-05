function [r_handles,o_handles] = getROhandles(clientID,vrep,p_dummies,o_dummies,r_robot,r_obstacle)


%%% Get the handle of jaco spheres
for i = 0:max(size(r_robot))-1
        [result, r_handles(i+1)] = vrep.simxGetObjectHandle(clientID, p_dummies(6*i+1:6*i+6), vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get handle for %s', p_dummies(6*i+1:6*i+6))
        end
end
%%% Get the handle of obstacles
for i = 0:max(size(r_obstacle))-1
        [result, o_handles(i+1)] = vrep.simxGetObjectHandle(clientID, o_dummies(6*i+1:6*i+6), vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get handle for %s', o_dummies(6*i+1:6*i+6))
        end
end

end