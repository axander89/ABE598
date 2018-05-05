function cost = COST(x_min,tree)

    cost = 0;
    p1 = tree(x_min).theta;
    parent = tree(x_min).parent;
    
    while parent~=0
        p2 = tree(parent).theta;
        cost = norm(p1-p2)+cost;
        p1 = p2;
        parent = tree(parent).parent;
    end

end