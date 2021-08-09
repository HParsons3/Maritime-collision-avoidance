%This function determines where each ship will need to move to, with
%regards to its current path and speed. It will plot the start and end
%points on the graph, as well as any turns the ship may have to take.

function [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, pathsize, ships, shipID, t)
        locationold(shipID,:) = location(shipID,:); %Set previous location
        location(shipID,:) = path(shipID,:,t); %Move along the path
        points = [0, 0];
        o = location(shipID,2)-locationold(shipID,2); 
        a = location(shipID,1)-locationold(shipID,1);
    %   h = sqrt(a^2+o^2); %Distance to target
        theta(shipID) = atand(o/a); 
        if a <= 0 && o <= 0 %Angle to target
            theta(shipID) = 270-theta(shipID);
        elseif a <= 0 && o >= 0
            theta(shipID) = 270-theta(shipID);
        elseif a >= 0 && o <= 0
            theta(shipID) = 90-theta(shipID);
        elseif a >= 0 && o >= 0
            theta(shipID) = 90-theta(shipID);
        end
        if location(shipID,:) == [0,0] %If the path is complete, remain in place
            location(shipID,:) = locationold(shipID,:);
        end
        if shipID > 1
            if locationold(shipID,:) == location(shipID,:) %If it is stationary, don't do anything
                points = location(shipID,:);
            else
                distance = sqrt(((locationold(shipID,1)-location(shipID,1))^2)+((locationold(shipID,2)-location(shipID,2))^2)); 
                if distance < speed(shipID)
                    overrun = speed(shipID)-distance;
                else
                    overrun = 0;
                end
                if overrun > 0.001
                    if location(shipID,:) == goalloc(shipID,:)
                        points = location(shipID,:);
                    else
                        ships = 1;
                        count = 1;
                        [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, shipID, ships, pathsize, t, path, goalloc);
                        if collision == 1
                            [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, pathsize, shipID, ships, t);
                            h2(ships) = plot(points(:,1), points(:,2), '-');
                        end
                        points = [locationold(shipID,:); location(shipID,:)];
                        while overrun > 0.001 && count <= 3
                            locationold(shipID,:) = location(shipID,:);
                            [~,~,pathmax] = size(path);
                            if t+count <= pathmax
                                locationdir = path(shipID,:,t+count);
                                o = locationdir(2)-locationold(shipID,2); 
                                a = locationdir(1)-locationold(shipID,1);
                            %   h = sqrt(a^2+o^2); %Distance to target
                                theta(shipID) = atand(o/a); 
                                if a <= 0 && o <= 0 %Angle to target
                                    theta(shipID) = 270-theta(shipID);
                                elseif a <= 0 && o >= 0
                                    theta(shipID) = 270-theta(shipID);
                                elseif a >= 0 && o <= 0
                                    theta(shipID) = 90-theta(shipID);
                                elseif a >= 0 && o >= 0
                                    theta(shipID) = 90-theta(shipID);
                                end
                                location(shipID,:) = [locationold(shipID,1)+(sind(theta(shipID))*overrun), locationold(shipID,2)+cosd(theta(shipID))*overrun];
                                [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, shipID, ships, pathsize, t, path, goalloc);
                                if collision == 1
                                    [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, pathsize, shipID, ships, t);
                                end
                                [location(shipID,:), overrun] = snapToTarget(location(shipID,:), path(shipID,:,t+count), theta(shipID));
                                points = [points; location(shipID,:)];
                                count = count+1;
                            else
                                location(shipID,:) = goalloc(shipID,:);
                                points = location(shipID,:);
                                overrun = 0;
                            end
                        end
                    end
                else
                count = 1;
                    for ships = 1:shipID-1 %For every ship that has already been computed
                        [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, shipID, ships, pathsize, t, path, goalloc);
                        if collision == 1
                            [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, pathsize, shipID, ships, t);
                            if points(height(points),:) ~= [0,0]
                                h2(ships) = plot(points(:,1), points(:,2), '-');
                            end
                        end
                    end
                    points = [locationold(shipID,:); location(shipID,:)];
                end
                if t > 1 && points(height(points),1) ~= 0
                    h2(shipID) = plot(points(:,1), points(:,2), '-');
                end

            end
        else
            distance = sqrt(((locationold(shipID,1)-location(shipID,1))^2)+((locationold(shipID,2)-location(shipID,2))^2)); 
                if distance < speed(shipID)
                    overrun = speed(shipID)-distance;
                else
                    overrun = 0;
                end
                if overrun > 0.001
                    if location(shipID,:) == goalloc(shipID,:)
                        points = location(shipID,:);
                    else
                        count = 1;
                        points = [locationold(shipID,:); location(shipID,:)];
                        while overrun > 0.001 && count <= 3
                            locationold(shipID,:) = location(shipID,:);
                            [~,~,pathmax] = size(path);
                            if t+count <= pathmax
                                locationdir = path(shipID,:,t+count);
                                o = locationdir(2)-locationold(shipID,2); 
                                a = locationdir(1)-locationold(shipID,1);
                            %   h = sqrt(a^2+o^2); %Distance to target
                                theta(shipID) = atand(o/a); 
                                if a <= 0 && o <= 0 %Angle to target
                                    theta(shipID) = 270-theta(shipID);
                                elseif a <= 0 && o >= 0
                                    theta(shipID) = 270-theta(shipID);
                                elseif a >= 0 && o <= 0
                                    theta(shipID) = 90-theta(shipID);
                                elseif a >= 0 && o >= 0
                                    theta(shipID) = 90-theta(shipID);
                                end
                                location(shipID,:) = [locationold(shipID,1)+(sind(theta(shipID))*overrun), locationold(shipID,2)+cosd(theta(shipID))*overrun];
                                [location(shipID,:), overrun] = snapToTarget(location(shipID,:), path(shipID,:,t+count), theta(shipID));
                                points = [points; location(shipID,:)];
                                count = count+1;
                            else
                                location(shipID,:) = goalloc(shipID,:);
                                points = location(shipID,:);
                                overrun = 0;
                            end
                        end
                    end
                else
                    for ships = 1:shipID-1 %For every ship that has already been computed
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, shipID, ships, pathsize, t, path, goalloc);
                    end
                    points = [locationold(shipID,:); location(shipID,:)];
                end
                if t > 1 && points(height(points),1) ~= 0
                    h2(shipID) = plot(points(:,1), points(:,2), '-');
                end
        end
end