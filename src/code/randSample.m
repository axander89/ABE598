function sample = randSample(m)

    loop = "true";

    while loop == "true"
        sample = -pi+2*pi*rand(m,1);
        sample(2) = -deg2rad(47) + (deg2rad(47)+deg2rad(313))*rand;
        sample(3) = -deg2rad(19) + (deg2rad(19)+deg2rad(341))*rand;
        t2 = rad2deg(sample(2));
        t3 = rad2deg(sample(3));
        if t2 < 47 || t2 > 313
            loop = "true";
        elseif t3 < 19 || t3 > 341
            loop = "true";
        else
            loop = "false";
        end
    end

end