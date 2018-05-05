    function [D, tree] = RRT(S,tree,sample,p_robot,r_robot,p_obstacle,r_obstacle)

    %=================================================================
    % RRT:This looks direclty at the nearest neighbor of the sample 
    % in the current tree and if obstacle free, it adds it to the tree
    %=================================================================

    %============
    % Nearest
    %============
    points = size(tree,2);
    D = inf;
    ID = -1;

    for i=1:points
        d = norm(sample-tree(i).theta);
        if d < D
            D = d;
            ID = i;
        end
    end
    
    %===============================
    % if obstacle free add to tree
    %===============================
    if line_collision(S,sample,tree(ID).theta,p_robot,r_robot,p_obstacle,r_obstacle) == 0
        node.theta = sample;
        node.id = tree(points).id+1;
        node.parent = tree(ID).id;
        tree = [tree node];
    else
        D = inf;
    end

end