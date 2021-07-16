%This is the simulation script. It creates a [mapsize]x[mapsize] map and
%generates [entitycount] number of entities (simulated vessels) and assigns
%each a goal. They will each generate a straight path to their goal
%initially, but if a collision risk is detected, they will divert their
%route using the collisionAvoid function, which utilises the RRT*
%algorithm.

% startloc = [randi(1000,1), randi(1000,1)];
% goalloc = [randi(1000,1), randi(1000,1)];
% clear all
% close all
% 
% entitycount = 20;
% mapsize = 50;
% rrt(startloc, goalloc);
% runSim(20,50)
clear all
close all

entitycount = 20;
mapsize = 50;
% rrt(startloc, goalloc);
launchSim(entitycount,mapsize)

function launchSim(entitycount, mapsize)
    count = 1;
%    startnode = [startloc, 0]; %Node structure is coordinates, and the parent
                            %node number in the tree array
%    starttree = startnode;
%    goalnode = [goalloc, 0];
%    goaltree = goalnode;
    [startloc, goalloc] = generateMap(entitycount, mapsize);
%    path = createPath(starttree, goaltree, count, map);
end

function [startloc, goalloc] = generateMap(entitycount, mapsize)
    %Preallocate arrays
    theta = zeros(entitycount,1); 
    location = zeros(entitycount,2);
    locationold = zeros(entitycount,2);
    startloc = rand(entitycount,2)*mapsize;
    goalloc = rand(entitycount,2)*mapsize;
    speed = (rand(entitycount,1)*2)+1;
    %Generate graph with vessel start points (Diamonds) and goals (rings)
    hold on 
    scatter(startloc(:,1),startloc(:,2),'d')
    scatter(goalloc(:,1),goalloc(:,2),'o')
    for n = 1:entitycount %For each entity
%        plot([startloc(n,1), goalloc(n,1)], [startloc(n,2), goalloc(n,2)])
        %Calculate distance and angle to goal using trigonometry
        o = goalloc(n,2)-startloc(n,2); 
        a = goalloc(n,1)-startloc(n,1);
%        h = sqrt(a^2+o^2); %Distance to target
        theta(n) = atand(o/a); 
        if a <= 0 && o <= 0 %Angle to target
            theta(n) = 270-theta(n);
        elseif a <= 0 && o >= 0
            theta(n) = 270-theta(n);
        elseif a >= 0 && o <= 0
            theta(n) = 90-theta(n);
        elseif a >= 0 && o >= 0
            theta(n) = 90-theta(n);
        end
    end
    %Generate a straight path from start location to goal
    path = zeros(entitycount, 2);
    i = 1;
    readyflag = 0;
    while readyflag == 0 && i < 100
        readyflag = 1;
        for n = 1:entitycount
            path(n,:,i) = [startloc(n,1)+sind(theta(n))*speed(n)*i, startloc(n,2)+cosd(theta(n))*speed(n)*i];
            if theta(n) >= 0 && theta(n) < 90 
                if path(n,1,i) >= goalloc(n,1) && path(n,2,i) >= goalloc(n,2)
                    path(n,:,i) = goalloc(n,:);
                else
                    readyflag = 0;
                end
            elseif theta(n) >= 90 && theta(n) < 180
                if path(n,1,i) >= goalloc(n,1) && path(n,2,i) <= goalloc(n,2)
                    path(n,:,i) = goalloc(n,:);
                else
                    readyflag = 0;
                end
            elseif theta(n) >= 180 && theta(n) < 270
                if path(n,1,i) <= goalloc(n,1) && path(n,2,i) <= goalloc(n,2)
                    path(n,:,i) = goalloc(n,:);
                else
                    readyflag = 0;
                end
            elseif theta(n) >= 270 && theta(n) < 360
                if path(n,1,i) <= goalloc(n,1) && path(n,2,i) >= goalloc(n,2)
                    path(n,:,i) = goalloc(n,:);
                else
                    readyflag = 0;
                end
            end
        end
        i = i+1;
    end
    %
    flag1 = 0;
    for t = 1:i-1 %Up to 50 ticks
        for n = 1:entitycount %For every entity
            if t > 1 %Start saving old locations after first tick
                locationold(n,:) = location(n,:);
                location(n,:) = path(n,:,t);
                if n > 1
                    if locationold(n,:) == location(n,:)
                    else
                        for j = 1:n-1
                            if locationold(j,:) == location(j,:)
                            else
                                intersection = findIntersection(locationold(n,:),location(n,:),locationold(j,:),location(j,:));
                                if intersection == 1
                                    if (theta(n)-theta(j) <= -190 && theta(n)-theta(j) > -315) || theta(n)-theta(j) >= 45 && theta(n)-theta(j) < 170
                                        intersecttype = 1; %Starboard: Ship n must move
                                    elseif abs(theta(n)-theta(j)) >= 170 && abs(theta(n)-theta(j)) < 190
                                        intersecttype = 2; %head-on: Both must move
                                    elseif abs(theta(n)-theta(j)) >= -45 && abs(theta(n)-theta(j)) < 45
                                        intersecttype = 3; %Overtake: ship j must move
                                    else
                                        intersecttype = 4; %Port: Ship j must move
                                    end
                                    flag1 = n;
                                    flag2 = j;
                                end
                            end
                        end
                    end
                end
            else
                location(n,:) = path(n,:,t);
            end
            if t > 1 %After first tick, start plotting lines to show movement during one tick
                h(n) = plot([location(n,1), locationold(n,1)], [location(n,2), locationold(n,2)]);
            end
%            locationy = startloc(n,2)+cosd(theta(n))*speed(n);
        end
%        scatter(location(:,1),location(:,2),'.') 
        if flag1 > 0
                    keyboard
                    flag1 = 0;
                    flag2 = 0;
        end
        if t > 1 %After first tick, start removing old lines
            delete(h(:));
        end
    end
end

%createPath now a part of collisionAvoid

% function path = createPath(tree1, tree2, count, map)
%     epsilon = 50;
%     random = [randi(1000,1), randi(1000,1)];
%     mindistance = 1000000;
%     for i = 1:count
% %        keyboard
%         distance = sqrt(((random(1)-tree1(i,1))^2)+((random(2)-tree1(i,2))^2));
%         if distance < mindistance
%             mindistance = distance;
%             near = i;
%         end
%     end
%     count = count+1;
%     x = random(1)-tree1(near,1);
%     y = random(2)-tree1(near,2);
%     if(mindistance > 50)
%         ratio = epsilon/mindistance;
%         x = (x*ratio);
%         y = (y*ratio);
%     end
%     newnode = [x+tree1(near,1),y+tree1(near,2),near];
%     if x >= 1 && y >= 1 && x <= 999 && y <= 999 && map(round(x),round(y)) == 1 
%         path = createPath(tree1, tree2, count, map);
%     else
%     tree1(count,:) = newnode;
%     distance = sqrt(((newnode(1)-tree2(1))^2)+((newnode(2)-tree2(2))^2));
%     if distance < 50
%         j = 1;
%         k = count;
%         while k ~= 0
%             path(j,:) = [tree1(k,1),tree1(k,2)];
%             j = j+1;
%             k = tree1(k,3);
%         end
%     else
%         path = createPath(tree1, tree2, count, map);
%     end
%     end
% end