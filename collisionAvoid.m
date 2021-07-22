%This function is called when a collision between two entities is
%anticipated. It draws a boundary between the two colliding vessels and
%uses RRT* to build a new path for the entity that must move.


function relaxedpath = collisionAvoid(locationA, locationB, goalA, goalB, count1, count2, tree1, tree2, flip)
    if count1 >= 5000
        keyboard
        path = [0,0];
    else
        count1
        if count1 == 1
            %keyboard
        end
%        epsilon = speedA; %Epsilon is the speed of the vessel
        epsilon = 0.25;
        random = [((rand()-0.5)*5)+tree1(1,1), ((rand()-0.5)*5)+tree1(1,2)]; %Find random location
        mindistance = 1000000; %Preallocate
        for i = 1:count1 %Check each node in tree to find closest
            distance = sqrt(((random(1)-tree1(i,1))^2)+((random(2)-tree1(i,2))^2)); 
            if distance < mindistance
                mindistance = distance; 
                near = i;
            end
        end
        x = random(1)-tree1(near,1); %Distance in x from nearest point
        y = random(2)-tree1(near,2); %Distance in y from nearest point
        ratio = epsilon/mindistance; %Create a new node in position in the direction from nearest node to random node
        if mindistance > epsilon
            x = (x*ratio);
            y = (y*ratio);
        end
        newnode = [x+tree1(near,1),y+tree1(near,2),near];
        %If the path between the nearest node and the new node crosses the
        %boundaries, then find a new point.
%        keyboard
        if findIntersection(tree1(near,:), newnode, locationA, locationB) == 1 || findIntersection(tree1(near,:), newnode, locationB, goalB) == 1
            path = collisionAvoid(locationA, locationB, goalA, goalB, count1, count2, tree1, tree2, flip); 
        else
            count1 = count1+1; %Create new node 
    %         if x >= 1 && y >= 1 && x <= 999 && y <= 999 && map(round(x),round(y)) == 1 
    %             path = createPath(tree1, tree2, countA, map);
    %         else
                tree1(count1,:) = newnode; %Add new node to tree
                mindistance = 1000000;
                for i = 1:count2
                    distance = sqrt(((newnode(1)-tree2(i,1))^2)+((newnode(2)-tree2(i,2))^2)); 
                    if distance < mindistance
                        if findIntersection(newnode, tree2(i,:), locationA, locationB) == 0 && findIntersection(newnode, tree2(i,:), locationB, goalB) == 0
                            mindistance = distance; 
                            near = i;
                        end
                    end
                end
               distance = sqrt(((newnode(1)-goalA(1))^2)+((newnode(2)-goalA(2))^2)); %Find distance to goal
                if mindistance < epsilon %If within 1 time frame
                    keyboard
                    if flip > 0
                        j = 1;
                        k = count1;
                        while k ~= 0
                            temppath(j,:) = [tree1(k,1),tree1(k,2)]; %Follow the tree to the root
                            j = j+1;
                            k = tree1(k,3);
                        end
    %                     if flip > 0
                        for a = 1:height(temppath)
                            path(a,:) = temppath((height(temppath)-a)+1,:);
                        end
    %                     end
                        path(j,:) = [tree2(near,1),tree2(near,2)];
                        j = j+1;
                        k = tree2(near,3);
                        while k ~= 0
                            path(j,:) = [tree2(k,1),tree2(k,2)];
                            j = j+1;
                            k = tree2(k,3);
                        end
                    else
                        j = 2;
                        temppath(1,:) = [tree2(near,1),tree2(near,2)];
                        k = tree2(near,3);
                        while k ~= 0
                            temppath(j,:) = [tree2(k,1),tree2(k,2)];
                            j = j+1;
                            k = tree2(k,3);
                        end
                        for a = 1:height(temppath)
                            path(a,:) = temppath((height(temppath)-a)+1,:);
                        end
                        k = count1;
                        path(j,:) = [tree1(count1,1),tree1(count1,2)];
                        j = j+1;
                        k = tree1(count1,3);
                        while k ~= 0
                            path(j,:) = [tree1(k,1),tree1(k,2)];
                            j = j+1;
                            k = tree1(k,3);
                        end
                    end
 %                   keyboard
                else
                    path = collisionAvoid(locationA, locationB, goalA, goalB, count2, count1, tree2, tree1, -flip); %Continue pathing
                end
    %         end
        end
    end
    forward = path(1,:);
    i = 1;
    j = 1;
    max = height(path);
    keyboard
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
    relaxedpath(i,:) = forward;
end