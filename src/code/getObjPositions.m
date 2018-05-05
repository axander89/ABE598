function q = getObjPositions(clientID,vrep,handles,n)

    for i = 0:n-1
        [result, q(:,i+1)] = vrep.simxGetObjectPosition(clientID,handles(i+1),-1,vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get position for %s', points(6*i+1:6*i+6))
        end
    end

end