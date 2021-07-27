function finalpath = relaxPath(path, locationA, locationB, goalA, goalB, speedA, speedB)
    forward = path(1,:); %Start at the first point in the path
    i = 1;
    j = 1;
    max = height(path);
    while j < max
        relaxedpath(i,:) = forward; %First point will always be the same
        backward = path(max,:); %Check the last point on the path
        n = max;
        relaxfound = 0;
        while n ~= 1 && relaxfound == 0
            %If there is a direct line from the start and end points
            %checked, raise flag
            if findIntersection(forward,backward,locationB,goalB) == 0 && findIntersection(forward,backward,locationA,locationB) == 0
                relaxfound = 1;
            %Otherwise, continue iterating backwards
            else
                n = n-1;
                backward = path(n,:);
            end
        end
        if relaxfound == 1 %If flag was raised, set the next start point to the end point
            forward = backward;
            j = n;
            i = i+1;
        else %Otherwise, check the next point in the path
            i = i+1;
            j = j+1;
            forward = path(i,:);
        end
    end
    relaxedpath(1,:) = []; %Delete the first point so it's not duplicated at the end
    n = 1;
    finalpath(:,:) = buildPath(locationA, relaxedpath(n,:),speedA); %Build a path to pay attention to the speed of the vessel
    while n < height(relaxedpath) %Build the rest of the path, paying attention to the vessel's speed
        [pathextra, ~] = buildPath(relaxedpath(n,:), relaxedpath(n+1,:),speedA);
        finalpath = [finalpath(:,:);pathextra(:,:)];
        n = n+1;
    end
    pathextra = buildPath(relaxedpath(n,:), goalA,speedA); %Path to the goal
    finalpath = [finalpath(:,:);pathextra(:,:)];
    finalpath(height(finalpath),:) = []; %Delete the goal point to prevent it duplicating
end