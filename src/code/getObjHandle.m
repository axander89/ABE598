function handles = getObjHandle(clientID,vrep,points,n)

    for i=0:n-1
        [result, handles(i+1)] = vrep.simxGetObjectHandle(clientID, points(6*i+1:6*i+6), vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get handle for %s', points(6*i+1:6*i+6))
        end
    end
    
end