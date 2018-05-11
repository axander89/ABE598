function X_near_id = NEAR(sample,tree,y,n)

    X_near_id = [];
    X_near_temp = [];
    points = size(tree,2);

    % look for nearests points
    for i=1:points
        d = norm(sample-tree(i).theta);
        D(i) = d;
        x_near.id = i;
        x_near.D = d;
        X_near_temp = [x_near X_near_temp];
    end

    D = sort(D);
    
    if points < y
        y = points;
    end
        
    for i=1:y
        for j=1:points
            if D(i) == X_near_temp(j).D
                X_near_id = [X_near_id X_near_temp(j).id];
                break
            end
        end
    end
    
end