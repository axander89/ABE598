%====================================================================
% Reset Session
%====================================================================
clear
clc

%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
p_dummies = ['Dumy00','Dummy0','Dummy1','Dummy2',...
             'Dummy3','Dummy4','Dummy5','Dummy6'];
% r_robot = [0.1 0.1 0.1 0.075 0.07 0.07 0.13 0.1 0.1 0.1]/2;
r_robot = [0.12,0.12,0.10,0.10,0.10,0.10,0.10,0.12]/2;
o_dummies = ['Dummy8','Dummy9','Dumy10','Dumy11'];
% r_obstacle = [0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2]/2;
r_obstacle = [0.15,0.15,0.15,0.15]/2;

%====================================================================
% Initialize session and Start Simulation
%====================================================================
[clientID, vrep] = InitSession(); pause(1)

%====================================================================
% Get handles
%====================================================================
% jaco handles
[joint_handles,jaco_handle,JacoHand_Dummy7] = getJacoHandles(clientID,vrep,ArmJoints);
% jaco spheres and obstacles handles
[r_handles,o_handles] = getROhandles(clientID,vrep,p_dummies,o_dummies,r_robot,r_obstacle);
% goal handle
[result, goal_handle] = vrep.simxGetObjectHandle(clientID, 'Goal', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Goal handle')
end

%====================================================================
% First obtain R and p for the base frame (Jaco) w.r.t the world frame
% and then S w.r.t the world frame
%====================================================================
%%% Get the position of base frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco position')
end
%%% Get the orientation of base w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco orientation')
end
R_base_in_world = getEulerOrientation(euler_angles(1),euler_angles(2),euler_angles(3));
p_base_in_world = p';




for K_loop =1:1

%====================================================================
% Obtain jaco and obstacles initial positions
%====================================================================
p_robot = getPositions(vrep,clientID,p_dummies,r_handles,8);
p_obstacle = getPositions(vrep,clientID,o_dummies,o_handles,4);
OpenJacoHand(vrep,clientID) % Open Jaco Hand
pause(2)

%====================================================================
% Find S = [S1 S2 ...]
% = = = = = = = = = =
% a,q without fk are w.r.t the base frame
% a_fk and q_fk are w.r.t the world frame
%====================================================================
S = getJacoS(clientID,vrep,joint_handles,jaco_handle,R_base_in_world,p_base_in_world);

%====================================================================
% Obtain the initial pose, M_1in0, of the end effector (dummy7 frame)
%====================================================================
%%% Get the position of dummy7 frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,JacoHand_Dummy7,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 position')
end
%%% Get the orientation of dummy7 w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, JacoHand_Dummy7, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 orientation')
end
p_end = p';
R_end = getEulerOrientation( euler_angles(1),euler_angles(2),euler_angles(3) );
M_1in0 = getH(R_end,p_end); %% ++++++ NEEDED +++++++ %%


%====================================================================
% Obtain the desired pose, M_2in0, of the end effector
% which is the same as the pose of the "Goal"
%====================================================================
%%% Get the position of Goal frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,goal_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Goal position')
end
%%% Get the orientation of Goal w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, goal_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Goal orientation')
end
p_2in0 = p';
R_2in0 = getEulerOrientation( euler_angles(1),euler_angles(2),euler_angles(3) );
M_2in0 = getH(R_2in0,p_2in0); %% ++++++ NEEDED +++++++ %%

%====================================================================
% Inv kinematics: FIND DESIRED ANGLES
%====================================================================
disp('================================')
disp('Performing Inverse Kinematics')
disp('================================')
theta_start = thetaStart(clientID,vrep,joint_handles);
loop = "true";
err = 1;
while loop == "true" || err == 1
    [theta_goal, err] = inverseKinematics(M_1in0,M_2in0,S);
    t2 = rad2deg( theta_start(2)+theta_goal(2) );
    t3 = rad2deg( theta_start(3)+theta_goal(3) );
    if err == 0
        if t2 < 47 || t2 > 313
            loop = "true";
        elseif t3 < 19 || t3 > 341
            loop = "true";
        else
            loop = "false";
        end
    end
end

%====================================================================
% Path Planning
%====================================================================
[success_RRT,q_RRT,iterations,npaths,npoints,opt_cost] = ABE598_PathPlanningRRT(S,zeros(6,1),theta_goal,p_robot,r_robot,p_obstacle,r_obstacle,2500);
[success_RRT_STAR,q_RRT_start,iterations,npaths,npoints,opt_cost] = ABE598_PathPlanningRRT_STAR(S,zeros(6,1),theta_goal,p_robot,r_robot,p_obstacle,r_obstacle,2500);

%====================================================================
% Robot Motion for RRT
%====================================================================
if success_RRT == 1
    q = q_RRT;
    points = size(q,2);
    disp('=============================================')
    disp('Starting Robot Motion for RRT')
    disp('=============================================')
    pause(5)
    for j=1:points-1

        theta_a = q(:,j);
        theta_b = q(:,j+1);
        D = norm(theta_b-theta_a);
        e = 0.1;
        n = 1+ceil(D/e);
        s = linspace(0,1,n);

        for k=1:max(size(s))
            q_curr = (1-s(k))*theta_a + s(k)*theta_b;
            for i = 1:6
                %%% Set the desired value of the current joint variable
                pause(0.05/(points-1))
                vrep.simxSetJointTargetPosition(clientID, joint_handles(i), theta_start(i) + q_curr(i), vrep.simx_opmode_oneshot);
            end
        end
    end
end


pause(5)
%====================================================================
% Reseting Robot to Initial Position
%====================================================================
disp('=============================================')
disp('Reseting Robot to Initial Position')
disp('=============================================')
for i = 1:6
    %%% Set the desired value of the current joint variable
    pause(0.1)
    vrep.simxSetJointTargetPosition(clientID, joint_handles(i), theta_start(i), vrep.simx_opmode_oneshot);
end

pause(5)
%====================================================================
% Robot Motion for RRT_STAR
%====================================================================
if success_RRT_STAR == 1
    q = q_RRT_start;
    points = size(q,2);
    disp('=============================================')
    disp('Starting Robot Motion for RRT_STAR')
    disp('=============================================')
    pause(5)
    for j=1:points-1

        theta_a = q(:,j);
        theta_b = q(:,j+1);
        D = norm(theta_b-theta_a);
        e = 0.1;
        n = 1+ceil(D/e);
        s = linspace(0,1,n);

        for k=1:max(size(s))
            q_curr = (1-s(k))*theta_a + s(k)*theta_b;
            for i = 1:6
                %%% Set the desired value of the current joint variable
                pause(0.05/(points-1))
                vrep.simxSetJointTargetPosition(clientID, joint_handles(i), theta_start(i) + q_curr(i), vrep.simx_opmode_oneshot);
            end
        end
    end
end

end

pause(5)
%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
