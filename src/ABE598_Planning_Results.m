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
             'Dummy3','Dummy4','Dummy5','Dummy6']; % robot spheres names
r_robot = [0.12,0.12,0.10,0.10,0.10,0.10,0.10,0.12]/2; % robot spheres radii
o_dummies = ['Dummy8','Dummy9','Dumy10','Dumy11']; % obstacles names
r_obstacle = [0.15,0.15,0.15,0.15]/2; % obstacles radii

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




for K_L =1:1
%====================================================================
% Obtain jaco and obstacles initial positions
%====================================================================
p_robot = getPositions(vrep,clientID,p_dummies,r_handles,8);
p_obstacle = getPositions(vrep,clientID,o_dummies,o_handles,4);
OpenJacoHand(vrep,clientID) % Open Jaco Hand

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
it = 1500; % Number of iterations per path planning
for K_loop = 1:5 % Number of paths plannings
[success(K_loop,1),q,iterations(K_loop,1),npaths(K_loop,1),npoints(K_loop,1),opt_cost(K_loop,1)] = ABE598_PathPlanningRRT(S,zeros(6,1),theta_goal,p_robot,r_robot,p_obstacle,r_obstacle,it);
[success_(K_loop,1),q,iterations_(K_loop,1),npaths_(K_loop,1),npoints_(K_loop,1),opt_cost_(K_loop,1)] = ABE598_PathPlanningRRT_STAR(S,zeros(6,1),theta_goal,p_robot,r_robot,p_obstacle,r_obstacle,it);
end

end

%====================================================================
% Plot Results
%====================================================================

z = 0;
z_ = 0;
for i=1:K_loop
    if opt_cost(i) == inf
        opt_cost(i) = 0;
        z = z+1;
    end
    if opt_cost_(i) == inf
        opt_cost_(i) = 0;
        z_ = z_+1;
    end
end


figure()
subplot(2,3,1)
plot(1:K_loop,npaths,'ro',1:K_loop,ones(1,K_loop)*ceil(sum(npaths)/K_loop),'r')
title('RRT results for 1000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('# of paths found in graph','average','Location','best')

subplot(2,3,2)
plot(1:K_loop,opt_cost,'k*',1:K_loop,ones(1,K_loop)*ceil(sum(opt_cost)/(K_loop-z)),'k')
title('RRT results for 1000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('cost of most feasible path','average','Location','best')

subplot(2,3,3)
plot(1:K_loop,npoints,'go',1:K_loop,ones(1,K_loop)*ceil(sum(npoints)/K_loop),'g')
title('RRT results for 1000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('# of points in path','average','Location','best')

subplot(2,3,4)
plot(1:K_loop,npaths_,'ro',1:K_loop,ones(1,K_loop)*ceil(sum(npaths_)/K_loop),'r')
title('RRT* results for 1000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('# of paths found in graph','average','Location','best')

subplot(2,3,5)
plot(1:K_loop,opt_cost_,'k*',1:K_loop,ones(1,K_loop)*ceil(sum(opt_cost_)/(K_loop-z_)),'k')
title('RRT* results for 5000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('cost of most feasible path','average','Location','best')

subplot(2,3,6)
plot(1:K_loop,npoints_,'go',1:K_loop,ones(1,K_loop)*ceil(sum(npoints_)/K_loop),'g')
title('RRT* results for 1000 iterations')
xlabel('ith path planning')
ylabel('value')
legend('# of points in path','average','Location','best')

pause(2)

%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);