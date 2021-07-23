function finalpath = relaxPath(path, locationA, locationB, goalA, goalB, speedA, speedB)
    forward = path(1,:);
    i = 1;
    j = 1;
    max = height(path);
    while j < max
        relaxedpath(i,:) = forward;
        backward = path(max,:);
        n = max;
        relaxfound = 0;
        while n ~= 1 && relaxfound == 0
            if findIntersection(forward,backward,locationB,goalB) == 0 && findIntersection(forward,backward,locationA,locationB) == 0
                relaxfound = 1;
            else
                n = n-1;
                backward = path(n,:);
            end
        end
        if relaxfound == 1
            forward = backward;
            j = n;
            i = i+1;
        else
            i = i+1;
            j = j+1;
            forward = path(i,:);
        end
    end
    relaxedpath(1,:) = [];
    n = 1;
    finalpath(:,:) = buildPath(locationA, relaxedpath(n,:),speedA);
    while n < height(relaxedpath)
        pathextra = buildPath(relaxedpath(n,:), relaxedpath(n+1,:),speedA);
        finalpath = [finalpath(:,:);pathextra(:,:)];
        n = n+1;
    end
    pathextra = buildPath(relaxedpath(n,:), goalA,speedA);
    finalpath = [finalpath(:,:);pathextra(:,:)];
    finalpath(height(finalpath),:) = [];
%    keyboard
%    relaxedpath(i,:) = forward;
end