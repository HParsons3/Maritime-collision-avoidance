% startloc = [randi(1000,1), randi(1000,1)];
% goalloc = [randi(1000,1), randi(1000,1)];
clear all
close all
% rrt(startloc, goalloc);
rrt()

function rrt
    count = 1;
%    startnode = [startloc, 0]; %Node structure is coordinates, and the parent
                            %node number in the tree array
%    starttree = startnode;
%    goalnode = [goalloc, 0];
%    goaltree = goalnode;
    [startloc, goalloc] = generateMap(10, 50);
    path = createPath(starttree, goaltree, count, map)
end

function [startloc, goalloc] = generateMap(entitycount, mapsize)
    map = zeros(mapsize);
    startloc = rand(entitycount,2)*mapsize;
    goalloc = rand(entitycount,2)*mapsize;
    speed = randi(5,entitycount,1);
    hold on
    scatter(startloc(:,1),startloc(:,2),'d')
    scatter(goalloc(:,1),goalloc(:,2),'o')
    for n = 1:entitycount
        plot([startloc(n,1), goalloc(n,1)], [startloc(n,2), goalloc(n,2)])
        o = goalloc(n,2)-startloc(n,2); 
        a = goalloc(n,1)-startloc(n,1);
        h = sqrt(a^2+o^2); %Distance to target
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
    for t = 0:50
        for n = 1:entitycount
            location(n,:) = [startloc(n,1)+sind(theta(n))*speed(n)*t, startloc(n,2)+cosd(theta(n))*speed(n)*t];
            if theta(n) >= 0 && theta(n) < 90
                if location(n,1) >= goalloc(n,1) && location(n,2) >= goalloc(n,2)
                    location(n,:) = goalloc(n,:);
                end
            end
            if theta(n) >= 90 && theta(n) < 180
                if location(n,1) >= goalloc(n,1) && location(n,2) <= goalloc(n,2)
                    location(n,:) = goalloc(n,:);
                end
            end
            if theta(n) >= 180 && theta(n) < 270
                if location(n,1) <= goalloc(n,1) && location(n,2) <= goalloc(n,2)
                    location(n,:) = goalloc(n,:);
                end
            end
            if theta(n) >= 270 && theta(n) < 360
                if location(n,1) <= goalloc(n,1) && location(n,2) >= goalloc(n,2)
                    location(n,:) = goalloc(n,:);
                end
            end
%            locationy = startloc(n,2)+cosd(theta(n))*speed(n);
        end
        scatter(location(:,1),location(:,2),'.')
        keyboard
    end
%    keyboard
end

function path = createPath(tree1, tree2, count, map)
    epsilon = 50;
    random = [randi(1000,1), randi(1000,1)];
    mindistance = 1000000;
    for i = 1:count
%        keyboard
        distance = sqrt(((random(1)-tree1(i,1))^2)+((random(2)-tree1(i,2))^2));
        if distance < mindistance
            mindistance = distance;
            near = i;
        end
    end
    count = count+1;
    x = random(1)-tree1(near,1);
    y = random(2)-tree1(near,2);
    if(mindistance > 50)
        ratio = epsilon/mindistance;
        x = (x*ratio);
        y = (y*ratio);
    end
    newnode = [x+tree1(near,1),y+tree1(near,2),near];
    if x >= 1 && y >= 1 && x <= 999 && y <= 999 && map(round(x),round(y)) == 1 
        path = createPath(tree1, tree2, count, map);
    else
    tree1(count,:) = newnode;
    distance = sqrt(((newnode(1)-tree2(1))^2)+((newnode(2)-tree2(2))^2));
    if distance < 50
        j = 1;
        k = count;
        while k ~= 0
            path(j,:) = [tree1(k,1),tree1(k,2)];
            j = j+1;
            k = tree1(k,3);
        end
    else
        path = createPath(tree1, tree2, count, map);
    end
    end
end