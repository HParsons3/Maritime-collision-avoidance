%This function determines, if there is an intersection, whether it is a
%head-on, starboard, port or rear collision. It calls several other
%functions to perform the path planning required for the different types of
%collision.

function [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, shipID, ships, pathsize, t, path, goalloc)
    intersection = findIntersection(locationold(shipID,:),location(shipID,:),locationold(ships,:),location(ships,:)); %Find if there is an intersection
    collision = 0;
    if intersection == 1 %If there is a possibility of collision
        collision = 1;
        if (theta(shipID)-theta(ships) >= 67.5 && theta(shipID)-theta(ships) < 157.5) || (theta(shipID)-theta(ships) >= -292.5 && theta(shipID)-theta(ships) < -202.5)
            %Starboard: Ship n must move
            location(ships,:) = locationold(ships,:); %Step back
            location(shipID,:) = locationold(shipID,:); 
            startnode = [location(shipID,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(shipID,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(shipID,:), location(ships,:), path(shipID,:,t+1), goalloc(ships,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(shipID,:), location(ships,:), path(shipID,:,t+1), goalloc(ships,:), speed(shipID), speed(ships)); %Smooth path
            [~,~,h] = size(path);
            for k = t+1:h-1 %integrate new path into the old path
                backuppath(k-t,:) = path(shipID,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(shipID,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(shipID,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):h-2+height(avoidpath)
                path(shipID,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end

        elseif abs(theta(shipID)-theta(ships)) >= 157.5 && abs(theta(shipID)-theta(ships)) < 202.5
            %head-on: Both must move
            location(ships,:) = locationold(ships,:); %Step back
            location(shipID,:) = locationold(shipID,:);
            startnode = [location(shipID,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(shipID,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(shipID,:), location(ships,:), path(shipID,:,t+1), goalloc(ships,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(shipID,:), location(ships,:), path(shipID,:,t+1), goalloc(ships,:), speed(shipID), speed(ships)); %Smooth path
            for k = t+1:pathsize-1 %integrate new path into the old path
                backuppath(k-t,:) = path(shipID,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(shipID,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(shipID,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):pathsize-2+height(avoidpath)
                path(shipID,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
            location(ships,:) = locationold(ships,:); %Step back
            location(shipID,:) = locationold(shipID,:);
            startnode = [location(ships,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(ships,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(ships,:), location(shipID,:), path(ships,:,t+1), path(shipID,:,t+1), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(ships,:), location(shipID,:), path(ships,:,t+1), goalloc(shipID,:), speed(ships), speed(shipID)); %Smooth path
            for k = t+1:pathsize-1 %integrate new path into the old path
                backuppath(k-t,:) = path(ships,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):pathsize-2+height(avoidpath)
                path(ships,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        elseif (abs(theta(shipID)-theta(ships)) >= 0 && abs(theta(shipID)-theta(ships)) < 67.5) || (abs(theta(shipID)-theta(ships)) >= 292.5 && abs(theta(shipID)-theta(ships)) < 360)
            %Overtake: ship j must move
            location(ships,:) = locationold(ships,:); %Step back
            location(shipID,:) = locationold(shipID,:);
            startnode = [location(ships,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(ships,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(ships,:), location(shipID,:), path(ships,:,t+1), goalloc(shipID,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(ships,:), location(shipID,:), path(ships,:,t+1), goalloc(shipID,:), speed(ships), speed(shipID)); %Smooth path
            for k = t+1:pathsize-1 %integrate new path into the old path
                backuppath(k-t,:) = path(ships,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):pathsize-2+height(avoidpath)
                path(ships,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        else
            %Port: Ship j must move
            location(ships,:) = locationold(ships,:); %Step back
            location(shipID,:) = locationold(shipID,:);
            startnode = [location(ships,:),0]; %Set starting and ending nodes for exploration tree
            endnode = [path(ships,:,t+1),0];
            avoidpath = [0,0];
            avoidpath = collisionAvoid(location(ships,:), location(shipID,:), path(ships,:,t+1), goalloc(shipID,:), 1, 1, startnode, endnode, 1); %Compute collision avoidance path
            avoidpath = relaxPath(avoidpath, location(ships,:), location(shipID,:), path(ships,:,t+1), goalloc(shipID,:), speed(ships), speed(shipID)); %Smooth path
            [~,~,h] = size(path);
            for k = t+1:h-1 %integrate new path into the old path
                backuppath(k-t,:) = path(ships,:,k);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t:(t+(height(avoidpath)-1))
                path(ships,:,k) = avoidpath(k-(t-1),:);
            end
            for k = t+(height(avoidpath)):h-2+height(avoidpath)
                path(ships,:,k) = backuppath(k+1-(t+height(avoidpath)),:);
            end
        end
        location(shipID,:) = path(shipID,:,t);
        location(ships,:) = path(ships,:,t);
        
    end
end