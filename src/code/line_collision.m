function t = line_collision(S,theta_a,theta_b,p_robot,r_robot,p_obstacle,r_obstacle)


    %=================================================================
    % Detects collision along a stright line
    %=================================================================
    
    D = norm(theta_b-theta_a);
    e = 0.1;
    n = 1+ceil(D/e);
    s = linspace(0,1,n);
    t = 0;
    
    for i=1:max(size(s))
        theta = (1-s(i))*theta_a + s(i)*theta_b;
        c = check_collision(S,theta,p_robot,r_robot,p_obstacle,r_obstacle);
        if c == 1
            t = s(i);
            return
        end
    end
    
end