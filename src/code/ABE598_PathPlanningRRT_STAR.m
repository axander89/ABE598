function [success,q,iterations,npaths,npoints,opt_cost] = ABE598_PathPlanningRRT_STAR(S,theta_start,theta_goal,p_robot,r_robot,p_obstacle,r_obstacle,it)

disp('================================')
disp('RRT* started path planning! ')
disp('================================')
%==============================================
% declare nodes for theta_start and theta_goal
%==============================================
node_start.theta = theta_start;
node_start.id = 1;
node_start.parent = 0;
node_goal.theta = theta_goal;
node_goal.id = 1;
node_goal.parent = 0;
%=================+
% initialize trees
%===================
tree_start = node_start;
tree_goal = node_goal;
success = 0;
d_tree_start = inf;
d_tree_goal = inf;
npaths = 0;
npoints = 0;
%===================
% initialize paths
%===================
paths_start = []; % points in tree_start completing a path
paths_goal = []; % points in tree_goal completing a path
opt_path_start = []; % optimal point in tree_start completing a path
opt_path_goal = []; % optimal point in tree_goal completing a path
opt_cost = inf; % current cost of paths, 'inf' since no path yet.
iterations = it; % number of iterations
%==================
% PATH PLANNING
%==================
for i=1:iterations

    % sample & check collision for sample
    m = size(theta_start,1);
    sample = randSample(m);

    % check sample is from FREE
    collision = check_collision(S,sample,p_robot,r_robot,p_obstacle,r_obstacle);

    % obtain smaller distance to tree with no straight line collision and insert in three
    if collision == 0
        [d_tree_start, tree_start] = RRT_STAR(S,tree_start,sample,p_robot,r_robot,p_obstacle,r_obstacle);
        [d_tree_goal, tree_goal] = RRT_STAR(S,tree_goal,sample,p_robot,r_robot,p_obstacle,r_obstacle);
    end
    
    % IF A SAMPLE CONECTS BOTH START AND GOAL TREES, 
    % ONE COMPLETE PATH HAS BEEN FOUND
    if d_tree_start ~= inf && d_tree_goal ~= inf
        success = 1;
        paths_start = [paths_start size(tree_start,2)];
        paths_goal = [paths_goal size(tree_goal,2)];
        d_tree_start = inf;
        d_tree_goal = inf;
        cur_cost = COST(size(tree_start,2),tree_start) + COST(size(tree_goal,2),tree_goal); % cost of path from start trhough sample to goal
        if cur_cost < opt_cost
            opt_cost = cur_cost;
            opt_path_start = size(tree_start,2);
            opt_path_goal = size(tree_goal,2);
        end
    end
    
end

%=======================================================================
% CONSTRUCT PATH: Q = [theta_start . . theta_t_optimal . . theta_goal]
%=======================================================================
if success
    n = opt_path_start;
    q_start = tree_start(n).theta;
    id = tree_start(n).parent;
    while id~=0
        q_start = [tree_start(id).theta q_start];
        id = tree_start(id).parent;
    end
    n = opt_path_goal;
    q_goal = tree_goal(n).theta;
    id = tree_goal(n).parent;
    q_goal = tree_goal(id).theta;
    id = tree_goal(id).parent;
    while id~=0
        q_goal = [q_goal tree_goal(id).theta];
        id = tree_goal(id).parent;
    end
    q = [q_start q_goal];
    
    npaths = size(paths_start,2);
    npoints = size(q,2);
    sprintf('in %d itertions RRT* found %d paths, and the most feasible contains %d points and has the lowest cost of %d', iterations,npaths,npoints,opt_cost)
    disp('Constructing lowest-cost path: ')
    disp(q)
else
    sprintf('RRT* failed to find a path at this time, number of iterations performed was %d', iterations)
    q = zeros(size(theta_start));
end

end