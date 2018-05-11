function [theta error] = inverseKinematics(M,R,S)

    [dummy,sz] = size(S);
    error = 0;
    theta = zeros(sz,1);
    k = 0;
    e = 5;
    mu = 1e-2;
    
    while e > 0.01
        T = find_fk(S,theta,M);
        V = logm(R*inv(T));
        v = unskew4(V);
        J = jacobian(S,theta);
        theta_dot = inv(J'*J + mu*eye(sz))*J'*v;
        theta = theta + theta_dot;
        k = k+1;
        e = norm(v);
        
        if k > 1000
            disp("NO SOLUTION!!")
            error = 1;
            return
        end
    end
end