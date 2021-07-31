function [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc)
    intersection = findIntersection(locationold(n,:),location(n,:),locationold(j,:),location(j,:)); %Find if there is an intersection
    collision = 0;
    if intersection == 1 %If there is a possibility of collision
        collision = 1;
        if (theta(n)-theta(j) <= -190 && theta(n)-theta(j) > -315) || theta(n)-theta(j) >= 45 && theta(n)-theta(j) < 170
            %Starboard: Ship n must move
            location(j,:) = locationold(j,:); %Step back
            location(n,:) = locationold(n,:); 
            startnode = [location(n,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(n,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(n,:), location(j,:), path(n,:,t+1), goalloc(j,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(n,:), location(j,:), path(n,:,t+1), goalloc(j,:), speed(n), speed(j)); %Smooth path
            [~,~,h] = size(path);
            for k = t+1:h-1 %integrate new path into the old path
                backuppath(k-t,:) = path(n,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(n,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(n,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):h-2+height(avoidpath)
                path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end

        elseif abs(theta(n)-theta(j)) >= 170 && abs(theta(n)-theta(j)) < 190
            %head-on: Both must move
            location(j,:) = locationold(j,:); %Step back
            location(n,:) = locationold(n,:);
            startnode = [location(n,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(n,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(n,:), location(j,:), path(n,:,t+1), goalloc(j,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(n,:), location(j,:), path(n,:,t+1), goalloc(j,:), speed(n), speed(j)); %Smooth path
            for k = t+1:i-1 %integrate new path into the old path
                backuppath(k-t,:) = path(n,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(n,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(n,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):i-2+height(avoidpath)
                path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
            location(j,:) = locationold(j,:); %Step back
            location(n,:) = locationold(n,:);
            startnode = [location(j,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(j,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(j,:), location(n,:), path(j,:,t+1), path(n,:,t+1), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(j,:), location(n,:), path(j,:,t+1), goalloc(n,:), speed(j), speed(n)); %Smooth path
            for k = t+1:i-1 %integrate new path into the old path
                backuppath(k-t,:) = path(j,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):i-2+height(avoidpath)
                path(j,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        elseif abs(theta(n)-theta(j)) >= -45 && abs(theta(n)-theta(j)) < 45
            %Overtake: ship j must move
            location(j,:) = locationold(j,:); %Step back
            location(n,:) = locationold(n,:);
            startnode = [location(j,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(j,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(j,:), location(n,:), path(j,:,t+1), goalloc(n,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(j,:), location(n,:), path(j,:,t+1), goalloc(n,:), speed(j), speed(n)); %Smooth path
            for k = t+1:i-1 %integrate new path into the old path
                backuppath(k-t,:) = path(j,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):i-2+height(avoidpath)
                path(n,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        else
            %Port: Ship j must move
            location(j,:) = locationold(j,:); %Step back
            location(n,:) = locationold(n,:);
            startnode = [location(j,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(j,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(j,:), location(n,:), path(j,:,t+1), goalloc(n,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(j,:), location(n,:), path(j,:,t+1), goalloc(n,:), speed(j), speed(n)); %Smooth path
            [~,~,h] = size(path);
            for k = t+1:h-1 %integrate new path into the old path
                backuppath(k-t,:) = path(j,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(j,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):h-2+height(avoidpath)
                path(j,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        end
        location(n,:) = path(n,:,t);
        location(j,:) = path(j,:,t);
        
    end
end