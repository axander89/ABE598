function T = getExOrientation(M,th,S)
    
    sz = max(size(th));
    E = eye(4);
    for i=1:sz
        E = E*expm(braket6(S(:,i))*th(i));
    end
    T = E*M;

end