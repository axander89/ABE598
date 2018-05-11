function S = getJacoS(clientID,vrep,joint_handles,jaco_handle,R_base_in_world,p_base_in_world)

%====================================================================
% Find S = [S1 S2 ...]
% = = = = = = = = = =
% a,q without fk are w.r.t the base frame
% a_fk and q_fk are w.r.t the world frame
%====================================================================
for i=1:6

[result, Euler] = vrep.simxGetObjectOrientation(clientID, joint_handles(i), jaco_handle, vrep.simx_opmode_blocking) ;
if result ~= vrep.simx_return_ok
	sprintf('could not get joint%d orieantatio', i)
end
[result, q] = vrep.simxGetObjectPosition(clientID, joint_handles(i), jaco_handle, vrep.simx_opmode_blocking) ;
if result ~= vrep.simx_return_ok
	sprintf('could not get joint%d position', i)
end
R =  getEulerOrientation( Euler(1), Euler(2), Euler(3) );
a = R(1:3,3);
[a_wf, q_wf] = Base2World(a,q',R_base_in_world,p_base_in_world);
S(:,i) = Sr(a_wf,q_wf); %% ++++++ NEEDED +++++++ %%

end

end