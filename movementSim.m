function [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, j, n, t)
%        keyboard
        locationold(n,:) = location(n,:); %Set previous location
        location(n,:) = path(n,:,t); %Move along the path
        points = [0, 0];
        o = location(n,2)-locationold(n,2); 
        a = location(n,1)-locationold(n,1);
    %   h = sqrt(a^2+o^2); %Distance to target
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
        if location(n,:) == [0,0] %If the path is complete, remain in place
            location(n,:) = locationold(n,:);
        end
        if n > 1
            if locationold(n,:) == location(n,:) %If it is stationary, don't do anything
                points = location(n,:);
            else
                distance = sqrt(((locationold(n,1)-location(n,1))^2)+((locationold(n,2)-location(n,2))^2)); 
                if distance < speed(n)
                    overrun = speed(n)-distance;
                else
                    overrun = 0;
                end
                if overrun > 0.001
                    if location(n,:) == goalloc(n,:)
                        points = location(n,:);
                    else
                        j = 1;
                        count = 1;
                        [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        if collision == 1
                            keyboard
                            [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, n, j, t);
%                             if h2(j)
%                                 delete(h2(j));
%                             end
                            h2(j) = plot(points(:,1), points(:,2), '-');
                        end
                        points = [locationold(n,:); location(n,:)];
                        while overrun > 0.001 && count <= 3
                            locationold(n,:) = location(n,:);
                            [~,~,pathmax] = size(path);
                            if t+count <= pathmax
                                locationdir = path(n,:,t+count);
                                o = locationdir(2)-locationold(n,2); 
                                a = locationdir(1)-locationold(n,1);
                            %   h = sqrt(a^2+o^2); %Distance to target
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
                                location(n,:) = [locationold(n,1)+(sind(theta(n))*overrun), locationold(n,2)+cosd(theta(n))*overrun];
                                [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                                if collision == 1
                                    [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, n, j, t);
                                end
                                [location(n,:), overrun] = snapToTarget(location(n,:), path(n,:,t+count), theta(n));
                                points = [points; location(n,:)];
                                count = count+1;
                            else
                                location(n,:) = goalloc(n,:);
                                points = location(n,:);
                                overrun = 0;
                            end
                        end
                    end
                else
                count = 1;
                    for j = 1:n-1 %For every ship that has already been computed
                        [path, locationold, location, collision] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        if collision == 1
                            keyboard
                            [points, locationold, location, path] = movementSim(locationold, location, speed, path, theta, goalloc, i, n, j, t);
%                             if t > 1
%                                 delete(h2(j));
%                             end
                            if points(height(points),:) ~= [0,0]
                                h2(j) = plot(points(:,1), points(:,2), '-');
                            end
                        end
                    end
                    points = [locationold(n,:); location(n,:)];
                end
                if t > 1 && points(height(points),1) ~= 0
                    h2(n) = plot(points(:,1), points(:,2), '-');
                end

            end
        else
            distance = sqrt(((locationold(n,1)-location(n,1))^2)+((locationold(n,2)-location(n,2))^2)); 
                if distance < speed(n)
                    overrun = speed(n)-distance;
                else
                    overrun = 0;
                end
                if overrun > 0.001
                    if location(n,:) == goalloc(n,:)
                        points = location(n,:);
                    else
                        count = 1;
%                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                        points = [locationold(n,:); location(n,:)];
                        while overrun > 0.001 && count <= 3
                            locationold(n,:) = location(n,:);
                            [~,~,pathmax] = size(path);
                            if t+count <= pathmax
                                locationdir = path(n,:,t+count);
                                o = locationdir(2)-locationold(n,2); 
                                a = locationdir(1)-locationold(n,1);
                            %   h = sqrt(a^2+o^2); %Distance to target
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
                                location(n,:) = [locationold(n,1)+(sind(theta(n))*overrun), locationold(n,2)+cosd(theta(n))*overrun];

        %                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                                [location(n,:), overrun] = snapToTarget(location(n,:), path(n,:,t+count), theta(n));
                                points = [points; location(n,:)];
                                count = count+1;
                            else
                                location(n,:) = goalloc(n,:);
                                points = location(n,:);
                                overrun = 0;
                            end
                        end
                    end
                else
                    for j = 1:n-1 %For every ship that has already been computed
                        [path, locationold, location] = interceptControl(locationold, location, speed, theta, n, j, i, t, path, goalloc);
                    end
                    points = [locationold(n,:); location(n,:)];
                end
                if t > 1 && points(height(points),1) ~= 0
                    h2(n) = plot(points(:,1), points(:,2), '-');
                end
        end
end