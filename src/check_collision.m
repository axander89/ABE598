function  collision = check_collision(S,theta,P_robot,R_robot,P_obstacle,R_obstacle)

    collision = 0;

    %=================================================================
    % Forward kinematics for specific sphere
    %=================================================================
    
    % Sphere 'Dummy0' is affected only by joint 1
    p0 = find_fk(S(:,1),theta(1),getH(eye(3),P_robot(:,2)));
    % Sphere 'Dummy1' 'Dummy2' and 'Dummy3' is affected by joint 1,2
    p1 = find_fk(S(:,1:2),theta(1:2),getH(eye(3),P_robot(:,3)));
    p2 = find_fk(S(:,1:2),theta(1:2),getH(eye(3),P_robot(:,4)));
    p3 = find_fk(S(:,1:2),theta(1:2),getH(eye(3),P_robot(:,5)));
    % Sphere 'Dummy4' is affected by joint 1,2,3
    p4 = find_fk(S(:,1:3),theta(1:3),getH(eye(3),P_robot(:,6)));
    % Sphere 'Dummy5' by joint 1,2,3,4
    p5 = find_fk(S(:,1:4),theta(1:4),getH(eye(3),P_robot(:,7)));
    % Sphere 'Dummy6' by joint 1,2,3,4,5,6
    p6 = find_fk(S(:,1:6),theta(1:6),getH(eye(3),P_robot(:,8)));
    
    q_robot = [P_robot(:,1) p0(1:3,4) p1(1:3,4) p2(1:3,4) p3(1:3,4) p4(1:3,4) p5(1:3,4) p6(1:3,4)];
    
    %=================================================================
    % Detect collision Self Collision and Object Collision
    %=================================================================
    
    q = [q_robot P_obstacle];
    r = [R_robot R_obstacle];
    k = size(q_robot,2);
    n = size(q,2);
    for i=1:k
        for j=i+1:n
            if( norm(q(:,i)-q(:,j)) <= r(i)+r(j))
                collision = 1;
                return
            end
        end
    end
    
    %=================================================================
    % Plot Resulting Pose
    %=================================================================
    %plotRobot(q_robot,R_robot,P_obstacle,R_obstacle)
end