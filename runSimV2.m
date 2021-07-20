clear all
close all

startA = [-0.05,2];
startB = [-0.05,0];
goalA = [0.05,0];
goalB = [0.05,2];
speedA = 0.25;
speedB = 0.25;
startloc = [startA;startB];
goalloc = [goalA;goalB];
speed = [speedA,speedB];
runSim2(startloc,goalloc,speed);

function runSim2(startloc, goalloc, speed)
    hold on
    xlim([-5,5])
    ylim([-5,5])
    scatter(startloc(1,1),startloc(1,2),'dr')
    scatter(startloc(2,1),startloc(2,2),'db')
    scatter(goalloc(1,1),goalloc(1,2),'or')
    scatter(goalloc(2,1),goalloc(2,2),'ob')
    for n = 1:2 %For each entity
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
    path = zeros(2);
    i = 1;
    readyflag = 0;
    while readyflag == 0 && i < 100
        readyflag = 1;
        for n = 1:2
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
    [~,~,d] = size(path);
    for n = 1:2
        for x = 1:d
            line(x,:) = [path(n,1,x),path(n,2,x)];
            if line(x,:) == [0,0]
                line(x,:) = line(x-1,:);
            end
        end
        h1(n) = plot(line(:,1),line(:,2),'.');
    end
    flag1 = 0;
    for t = 1:(100-1) %Until the last path is finished, or it times out
        for n = 1:2 %For every entity
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
                                        keyboard
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
                                        keyboard
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
                                        keyboard
                                        avoidpath = collisionAvoid(location(j,:), location(n,:), speed(j), speed(n), path(j,:,t+1), path(n,:,t+1), 1, startnode);
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
                                        keyboard
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
                                        avoidpath = [0,0];
                                        watchdog = 1;
                                        while height(avoidpath) == 1 && watchdog < 50
                                            keyboard
                                            avoidpath = collisionAvoid(location(j,:), location(n,:), speed(j), speed(n), path(j,:,t+1), goalloc(n,:), 1, startnode);
                                            watchdog = watchdog+1;
                                     %       keyboard
                                        end
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
                                            path(j,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
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