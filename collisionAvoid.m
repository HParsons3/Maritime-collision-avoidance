%This function is called when a collision between two entities is
%anticipated. It draws a boundary between the two colliding vessels and
%uses RRT* to build a new path for the entity that must move.


function path = collisionAvoid(locationA, locationB, speedA, speedB, goalA, goalB, count, tree1)
    if count >= 10000
        path = [0,0];
    else
        count
        if count == 1
            keyboard
        end
%        epsilon = speedA; %Epsilon is the speed of the vessel
        epsilon = 0.25;
        random = [((rand()-0.5)*50)+tree1(1,1), ((rand()-0.5)*50)+tree1(1,2)]; %Find random location
        mindistance = 1000000; %Preallocate
        for i = 1:count %Check each node in tree to find closest
            distance = sqrt(((random(1)-tree1(i,1))^2)+((random(2)-tree1(i,2))^2)); 
            if distance < mindistance
                mindistance = distance; 
                near = i;
            end
        end
        x = random(1)-tree1(near,1); %Distance in x from nearest point
        y = random(2)-tree1(near,2); %Distance in y from nearest point
        ratio = epsilon/mindistance; %Create a new node in position in the direction from nearest node to random node
        x = (x*ratio);
        y = (y*ratio);
        newnode = [x+tree1(near,1),y+tree1(near,2),near];
        %If the path between the nearest node and the new node crosses the
        %boundaries, then find a new point.
%        keyboard
        if findIntersection(tree1(near,:), newnode, locationA, locationB) == 1 || findIntersection(tree1(near,:), newnode, locationB, goalB) == 1
            path = collisionAvoid(locationA, locationB, speedA, speedB, goalA, goalB, count, tree1); 
        else
            count = count+1; %Create new node 
    %         if x >= 1 && y >= 1 && x <= 999 && y <= 999 && map(round(x),round(y)) == 1 
    %             path = createPath(tree1, tree2, count, map);
    %         else
                tree1(count,:) = newnode; %Add new node to tree
                distance = sqrt(((newnode(1)-goalA(1))^2)+((newnode(2)-goalA(2))^2)); %Find distance to goal
                if distance < speedA %If within 1 time frame
                    j = 1;
                    k = count;
                    while k ~= 0
                        temppath(j,:) = [tree1(k,1),tree1(k,2)]; %Follow the tree to the root
                        j = j+1;
                        k = tree1(k,3);
                    end
 %                   keyboard
                    for a = 1:height(temppath)
                            path(a,:) = temppath((height(temppath)-a)+1,:);
                    end
                else
                    path = collisionAvoid(locationA, locationB, speedA, speedB, goalA, goalB, count, tree1); %Continue pathing
                end
    %         end
        end
    end
    keyboard
end