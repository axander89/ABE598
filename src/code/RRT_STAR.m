    function [D, tree] = RRT_STAR(S,tree,sample,p_robot,r_robot,p_obstacle,r_obstacle)

    %====================================================================
    % RRT_star:This looks direclty at the nearest neighbor of the sample 
    % in the current tree and if obstacle free, and if minimu cost
    % it adds it to the tree, and rewires trees based on lower cost
    %===================================================================

    %============
    % Nearest
    %============
    points = size(tree,2);
    D = inf;
    id_nearest = -1;

    for i=1:points
        d = norm(sample-tree(i).theta);
        if d < D
            D = d;
            id_nearest = i;
        end
    end
    
    %=======================================================
    % if obstacle free and minimun cost add to tree
    %=======================================================
    if line_collision(S,sample,tree(id_nearest).theta,p_robot,r_robot,p_obstacle,r_obstacle) == 0
        
        X_near_id = NEAR(sample,tree,5,10);
        x_min = tree(id_nearest).theta;
        c_min = COST(id_nearest,tree) + norm(x_min-sample);
        id_min = id_nearest;
        
        % Connect along minimun cost path
        for i=1:size(X_near_id)
            Total_cost = COST(X_near_id(i),tree)+norm(tree(X_near_id(i)).theta-sample);
            if Total_cost < c_min
                if line_collision(S,sample,tree(X_near_id(i)).theta,p_robot,r_robot,p_obstacle,r_obstacle) == 0
                    c_min = Total_cost;
                    id_min = X_near_id(i);
                end
            end
        end
        node.theta = sample;
        node.id = tree(points).id+1;
        node.parent = tree(id_min).id;
        tree = [tree node];
        
        % Rewire tree
        for i=1:size(X_near_id)
            Total_cost = COST(node.id,tree)+norm(sample-tree(X_near_id(i)).theta);
            if Total_cost < COST(X_near_id(i),tree)
                if line_collision(S,sample,tree(X_near_id(i)).theta,p_robot,r_robot,p_obstacle,r_obstacle) == 0
                    tree(X_near_id(i)).parent = node.id;
                end
            end
        end
        
    else
        D = inf;
    end

end