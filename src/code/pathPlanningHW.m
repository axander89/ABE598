function q = pathPlanningHW(S,theta_start,theta_goal,p_robot,r_robot,p_obstacle,r_obstacle)

% declare nodes for theta_start and theta_goal
node_start.theta = theta_start;
node_start.id = 1;
node_start.parent = 0;
node_goal.theta = theta_goal;
node_goal.id = 1;
node_goal.parent = 0;
% initialize trees
tree_start = node_start;
tree_goal = node_goal;
% PATH PLANNING
d_tree_start = inf;
d_tree_goal = inf;
k = 0;
while d_tree_start == inf || d_tree_goal == inf

% sample & check collision for sample
m = size(theta_start,1);
sample = randSanple(m);

%check collision
collision = check_collision(S,sample,p_robot,r_robot,p_obstacle,r_obstacle);

% obtain smaller distance to tree with no straight line collision and
% insert in three
if collision == 0
    [d_tree_start, tree_start] = insert_in_tree(S,tree_start,sample,p_robot,r_robot,p_obstacle,r_obstacle);
    [d_tree_goal, tree_goal] = insert_in_tree(S,tree_goal,sample,p_robot,r_robot,p_obstacle,r_obstacle);
end
k = k+1;
if k > 100
    q = zeros(size(theta_start));
    tree_start
    tree_goal
    disp('No Path')
    return
end
end

% construct q = [theta_start . . . theta_goal]
[m,n] = size(tree_start);
q_start = tree_start(n).theta;
id = tree_start(n).parent;
while id~=0
    q_start = [tree_start(id).theta q_start];
    id = tree_start(id).parent;
end
[m,n] = size(tree_goal);
q_goal = tree_goal(n).theta;
id = tree_goal(n).parent;
q_goal = tree_goal(id).theta;
id = tree_goal(id).parent;
while id~=0
    q_goal = [q_goal tree_goal(id).theta];
    id = tree_goal(id).parent;
end

q = [q_start q_goal];

end