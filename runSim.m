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

entitycount = 10;
mapsize = 20;
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
    for t = 1:(100-1) %Until the last path is finished, or it times out
        for n = 1:entitycount %For every entity
            if t > 1 %Start saving old locations after first tick
                locationold(n,:) = location(n,:);
                location(n,:) = path(n,:,t);
                if location(n,:) == [0,0]
                    location(n,:) = locationold(n,:);
                end
                if n > 1
                    if locationold(n,:) == location(n,:)
                    else
                        for j = 1:n-1
                            if locationold(j,:) == location(j,:)
                            else
                                intersection = findIntersection(locationold(n,:),location(n,:),locationold(j,:),location(j,:));
                                if intersection == 1
%                                    keyboard
                                    if (theta(n)-theta(j) <= -190 && theta(n)-theta(j) > -315) || theta(n)-theta(j) >= 45 && theta(n)-theta(j) < 170
                                        intersecttype = 1; %Starboard: Ship n must move
                                        location(n,:) = locationold(n,:); %Step back
                                        startnode = [location(n,:),0];
                                        avoidpath = collisionAvoid(location(n,:), location(j,:), speed(n), speed(j), path(n,:,t+1), goalloc(j,:), 1, startnode);
%                                        keyboard
                                        for k = t+1:i-1
                                            backuppath(k-t,:) = path(n,:,k);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(n,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(n,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t+(height(avoidpath)):i-2+height(avoidpath)
 %                                           keyboard
                                            path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
                                        end
                                        
                                    elseif abs(theta(n)-theta(j)) >= 170 && abs(theta(n)-theta(j)) < 190
                                        location(n,:) = locationold(n,:); %Step back
                                        startnode = [location(n,:),0];
                                        avoidpath = collisionAvoid(location(n,:), location(j,:), speed(n), speed(j), path(n,:,t+1), goalloc(j,:), 1, startnode);
%                                        keyboard
                                        for k = t+1:i-1
                                            backuppath(k-t,:) = path(n,:,k);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(n,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(n,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t+(height(avoidpath)):i-2+height(avoidpath)
%                                            keyboard
                                            path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
                                        end
                                        location(j,:) = locationold(j,:); %Step back
                                        startnode = [location(j,:),0];
                                        avoidpath = collisionAvoid(location(j,:), location(n,:), speed(j), speed(n), path(j,:,t+1), goalloc(n,:), 1, startnode);
%                                        keyboard
                                        for k = t+1:i-1
                                            backuppath(k-t,:) = path(j,:,k);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t+(height(avoidpath)):i-2+height(avoidpath)
%                                            keyboard
                                            path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
                                        end
                                        intersecttype = 2; %head-on: Both must move
                                    elseif abs(theta(n)-theta(j)) >= -45 && abs(theta(n)-theta(j)) < 45
                                        intersecttype = 3; %Overtake: ship j must move
                                        location(j,:) = locationold(j,:); %Step back
                                        startnode = [location(j,:),0];
                                        avoidpath = collisionAvoid(location(j,:), location(n,:), speed(j), speed(n), path(j,:,t+1), goalloc(n,:), 1, startnode);
%                                        keyboard
                                        for k = t+1:i-1
                                            backuppath(k-t,:) = path(j,:,k);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t+(height(avoidpath)):i-2+height(avoidpath)
%                                            keyboard
                                            path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
                                        end
                                    else
                                        intersecttype = 4; %Port: Ship j must move
                                        location(j,:) = locationold(j,:); %Step back
                                        startnode = [location(j,:),0];
                                        avoidpath = collisionAvoid(location(j,:), location(n,:), speed(j), speed(n), path(j,:,t+1), goalloc(n,:), 1, startnode);
%                                        keyboard
                                        for k = t+1:i-1
                                            backuppath(k-t,:) = path(j,:,k);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t:(t+(height(avoidpath)-1))
                                            path(j,:,k) = avoidpath(k-(t-1),:);
                                        end
                                        for k = t+(height(avoidpath)):i-2+height(avoidpath)
%                                            keyboard
                                            path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
                                        end
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
                h1(n) = plot([location(n,1), locationold(n,1)], [location(n,2), locationold(n,2)]);
                [~,~,d] = size(path);
                for x = 1:d
                    line(x,:) = [path(n,1,x),path(n,2,x)];
                    if line(x,:) == [0,0]
                        line(x,:) = line(x-1,:);
                    end
                end
                h2(n) = plot(line(:,1),line(:,2),'.');
            end
%            locationy = startloc(n,2)+cosd(theta(n))*speed(n);
        end
        keyboard
%        scatter(location(:,1),location(:,2),'.') 
        if flag1 > 0
                    flag1 = 0;
                    flag2 = 0;
        end
        if t > 1 %After first tick, start removing old lines
            delete(h1(:));
            delete(h2(:));
        end
    end
end